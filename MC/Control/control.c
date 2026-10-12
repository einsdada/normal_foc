/**
 * @file    control.c
 * @brief   控制层实现：分频调度 + 三个频率环路 + 各模式处理
 *
 * 层级：User → Control → Device/BSP。
 *
 * 文件结构（按需跳转）：
 *   1. 实现常量        —— 只有本文件用的数学常量（可调参数全在 control.h）
 *   2. 内部工具        —— Foc_ObserveDq / Foc_GetSinCos / EncCalib_Reset
 *   3. 各模式处理      —— 一个模式一个函数，高频环每拍调用一个：
 *                          Mode_Stop / Mode_CurrentCalib / Mode_EncoderCalib /
 *                          Mode_FullOpenLoop / Mode_HalfOpenLoop / Mode_CurrentLoop
 *                          （Mode_SpeedCurrentLoop 在中频环执行，见第 3 节末尾）
 *                          编码器校准的**算法**在 Algorithm/mc_enc_calib.c（两种方式：
 *                          两点锁轴 / 慢速旋转，由 control.h 的 ENC_CALIB_METHOD 选），
 *                          本文件的 Mode_EncoderCalib 只是"读角度→调算法→写设备"的胶水
 *   4. 对外接口        —— Control_Init / Control_Sched / Control_HighLoop
 *                          / Control_MidLoop / Control_LowLoop
 *
 * 约定：
 *  - 所有状态统一封装在 MC_Handle_t（模式/目标/当前值/设备句柄/调度状态），
 *    函数统一注入 MC_Handle_t *handle，只操作 handle 指向的共享数据；
 *  - 每个环路先做【传感器信息更新】（switch 外、任何模式都运行，保证外部始终可读），
 *    再按 handle->mode 分发：
 *      Control_HighLoop（20kHz 每拍）  电流更新 + 模式处理
 *      Control_MidLoop （10kHz 2 分频）位置/角度/速度更新 + 速度环
 *      Control_LowLoop （5kHz  4 分频）位置环（待实现）
 *  - 单位：电流 A；角度 rad；Ud/Uq 与三相占空比为调制比（1.0 = Udc/2，非 V）。
 */
#include <stddef.h>     /* NULL */
#include "control.h"
#include "foc.h"         /* MC_SVPWM_Calc / MC_Clarke / MC_Park */
#include "mc_math.h"     /* MC_Math_Sin / MC_Math_Cos（查表法正余弦） */
#include "mc_speed.h"    /* SpeedLoop_Run / Reset（速度环 → Iq 给定） */

/* ==================== 1. 实现常量 ====================
 * 可调参数（校准方式/锁轴电压/稳定判据/方向判据/PI 初值）统一放在 control.h，此处只留
 * 本文件内部用的数学常量，以及"把 control.h 的宏折成算法层参数"这一步折算
 * （不对外暴露通用名 FOC_2PI/FOC_PI，避免与 foc.h/mc_math.h 撞名） */
#define FOC_2PI    6.283185307179586f   /* 2π：开环电角度积分步长换算 */
#define FOC_PI     3.141592653589793f   /* π：校准判据阈值的角度→弧度换算 */

/* ---- 编码器校准算法参数：control.h 的"人读"宏 → mc_enc_calib 要的形式 ----
 * 时间类参数在这里就折成拍数（算法里只数拍，省掉运行期乘除），角度阈值统一换成 rad。
 * 全部是编译期常量 ⇒ 直接塞进 static const 结构体；算法层因此不必认识 control.h 的宏名，
 * 也就保持了"Algorithm 不依赖 Control"的分层（与 mc_current/mc_speed 通过入参拿参数同理） */
#define ENC_ROT_RAMP_TICKS      ((uint16_t)(ENC_ROT_RAMP_S        * (float)CONTROL_HIGH_HZ))
#define ENC_ROT_SETTLE_TICKS    ((uint16_t)(ENC_ROT_SETTLE_S      * (float)CONTROL_HIGH_HZ))
#define ENC_ROT_ALIGN_MIN_TICKS ((uint16_t)(ENC_ROT_ALIGN_MIN_S   * (float)CONTROL_HIGH_HZ))
#define ENC_ROT_ALIGN_MAX_TICKS ((uint16_t)(ENC_ROT_ALIGN_MAX_S   * (float)CONTROL_HIGH_HZ))
#define ENC_ROT_WIN_TICKS       ((uint16_t)(ENC_ROT_WIN_S         * (float)CONTROL_HIGH_HZ))
#define ENC_ROT_MAX_MEAS_TICKS  ((uint16_t)(ENC_ROT_MAX_MEAS_S    * (float)CONTROL_HIGH_HZ))
#define DRV_FAULT_WAIT_TICKS    ((uint16_t)(DRV_FAULT_WAIT_S      * (float)CONTROL_HIGH_HZ))
/* 每个测量段要走够的电角度：REVS 个机械圈 × 极对数 × 2π */
#define ENC_ROT_TRAVEL_TARGET  (FOC_2PI * (float)MOTOR_POLE_PAIRS * ENC_ROT_REVS)
#define ENC_ROT_MAX_LOAD_RAD   (ENC_ROT_MAX_LOAD_DEG * (FOC_PI / 180.0f))

static const EncCalib_Cfg_t g_enc_calib_cfg = {
    .method        = ENC_CALIB_METHOD,
    .dir_force     = (int8_t)ENC_CALIB_DIR_FORCE,
    .pole_pairs    = (uint16_t)MOTOR_POLE_PAIRS,

    /* 方式一 两点锁轴 */
    .lock_volt     = ENC_CALIB_LOCK_VOLT,
    .min_settle    = (uint16_t)ENC_CALIB_MIN_SETTLE,
    .max_settle    = (uint16_t)ENC_CALIB_MAX_SETTLE,
    .read_n        = (uint16_t)ENC_CALIB_READ_N,
    .stable_err    = ENC_CALIB_STABLE_ERR,
    .dir_min_ratio = ENC_CALIB_DIR_MIN_RATIO,
    .dir_max_ratio = ENC_CALIB_DIR_MAX_RATIO,

    /* 方式二 慢速旋转 */
    .rot_volt            = ENC_ROT_VOLT,
    .rot_freq            = ENC_ROT_FREQ_HZ,
    .rot_ramp_ticks      = ENC_ROT_RAMP_TICKS,
    .rot_settle_ticks    = ENC_ROT_SETTLE_TICKS,
    .rot_align_min_ticks = ENC_ROT_ALIGN_MIN_TICKS,
    .rot_align_max_ticks = ENC_ROT_ALIGN_MAX_TICKS,
    .rot_win_ticks       = ENC_ROT_WIN_TICKS,
    .rot_max_meas_ticks  = ENC_ROT_MAX_MEAS_TICKS,
    .rot_travel_target   = ENC_ROT_TRAVEL_TARGET,
    .rot_ratio_min       = ENC_ROT_RATIO_MIN,
    .rot_ratio_max       = ENC_ROT_RATIO_MAX,
    .rot_max_load        = ENC_ROT_MAX_LOAD_RAD,
    .rot_retry_n         = (uint8_t)ENC_ROT_RETRY_N
};

/* ==================== 2. 内部工具 ==================== */

/**
 * @brief  三相电流 → 编码器电角度坐标系下的 Id/Iq（仅观测，不参与闭环）
 * @param  handle  数据库句柄
 * @param  s, c    编码器电角度的 sin/cos（调用方已算好，避免重复查表）
 * @note   结果写入 handle->state.id / state.iq，上位机看 ch7/ch8 即此值
 */
static void Foc_ObserveDq(MC_Handle_t *handle, float s, float c)
{
    float ialpha, ibeta;

    MC_Clarke(CurrentSensor_GetIa(&handle->device.cur),
              CurrentSensor_GetIb(&handle->device.cur),
              CurrentSensor_GetIc(&handle->device.cur),
              &ialpha, &ibeta);
    MC_Park(ialpha, ibeta, s, c, &handle->state.id, &handle->state.iq);
}

/**
 * @brief  取电流环/Park 用的电角度 sin/cos（含**角度延迟前推补偿**）
 * @param  handle  数据库句柄
 * @param  we      电角速度（rad/s，正 = 电角度增大方向；直接取 PosSensor_GetElecSpeed()）
 * @param  s, c    输出：补偿后的 sin/cos
 * @note   θ_used = θ_meas + ωe·ENC_ANGLE_LAG_US：编码器角度是"过去某一时刻"的
 *         （SSI 上一帧 + 中频环 10kHz 刷新 + PWM 生效延迟 ≈ 100~200us），转速越高
 *         滞后越大。不补偿时矢量在真实转子上被掰偏 ωe·Δt，转矩少 cos(...)、还要
 *         多输出 d 轴电压（实测顶速 Ud=−0.24 就是它）。推导与标定方法见 control.h
 */
static void Foc_GetSinCos(MC_Handle_t *handle, float we, float *s, float *c)
{
    float ang = PosSensor_GetElecAngle(&handle->device.pos)
              + we * (ENC_ANGLE_LAG_US * 1.0e-6f);

    MC_Math_SinCos(ang, s, c);      /* 一次算 sin/cos（省一次角度归一化，原 60 条 → ~35 条） */
}

/* 注：编码器校准的解绕、角度归一与两套状态机都已搬到 Algorithm/mc_enc_calib.c（算法层），
   本文件只留"读角度 → 调算法 → 写设备/落库"的胶水，见第 3 节的 Mode_EncoderCalib */

/* ==================== 3. 各模式处理（高频环 20kHz 每拍） ==================== */

/**
 * @brief  停止：关断输出
 * @note   幂等，每拍都执行——保证从任意模式切到停止必定关断
 */
static void Mode_Stop(MC_Handle_t *handle)
{
    MotorPwm_Stop(&handle->device.pwm);     /* 占空比清零 + 关断驱动器（设备层安全态） */
}

/**
 * @brief  电流偏置校准：0% 占空比 + 使能驱动器，零电流累加求中点
 * @note   PWM 0%（下桥臂全通 → 绕组短路 → 零电流）；必须使能驱动器，因为
 *         DRV8311 关断（sleep 低）时 SOx 电流检测输出无有效零点，bias 会采错；
 *         再次校准只需重新进入本模式（CurrentSensor_Calib 内部自动重置重新开始）；
 *         校准完成后：已有编码器偏移/方向 → 直接回停止；否则自动进入编码器校准
 */
static void Mode_CurrentCalib(MC_Handle_t *handle)
{
    MotorPwm_SetDuty(&handle->device.pwm, 0.0f, 0.0f, 0.0f);   /* 0% 占空比 = 三相下桥臂全通 */
    MotorPwm_EnsureEnabled(&handle->device.pwm);                /* 使能驱动器：SOx 零点有效 */
    CurrentSensor_Calib(&handle->device.cur);

    if (handle->device.cur.calib_done)
    {
        EncCalib_Reset(&handle->enc_calib);     /* 校准状态机复位到整段最开头（算法层） */
        handle->mode = handle->enc_calib_done ? MODE_STOP : MODE_ENCODER_CALIB;
    }
}

/**
 * @brief  编码器校准（模式处理）：读机械角 → 跑 Algorithm 的校准算法 → 写设备/落库
 * @note   算法本体（两套状态机、解绕、方向判据、阈值判据）在 Algorithm/mc_enc_calib.c；
 *         本函数只做三件事，写法与 Mode_HalfOpenLoop 一致：
 *           1) 把 PosSensor 的**原始**机械角与本拍 dt 交给 EncCalib_Run；
 *           2) 未完成 → 使能驱动器 + 把算法给的 dq 矢量经 SVPWM 写进 PWM；
 *           3) 完成 → 结果写回 pos 句柄（**方向必须先于偏移生效**）、关断、
 *              置一次性上报标志、回停止
 *         参数由本文件顶部的 g_enc_calib_cfg 折算而来，control.h 仍是唯一的调参处
 */
static void Mode_EncoderCalib(MC_Handle_t *handle)
{
    EncCalib_Out_t out;
    float u, v, w;

    /* ---- 驱动器故障（nFAULT 拉低）：输出级被芯片自己关断 ⇒ 没电流就没转矩 ----
       硬跑校准毫无意义（转子纹丝不动 ⇒ dm=0 ⇒ 被误判成丢步、白跑重试），
       但也不能一故障就永久放弃（可能是暂态：电源跌落/干扰/过流自动重试）。
       策略：故障期间只等待、不推进校准；恢复后复位重来；累计太久才放弃。

       ⚠⚠ 等待期间**只清占空比，绝不能调用 MotorPwm_Stop()**：
         Stop 的语义是"清零 + 关断驱动器（nSLEEP 拉低）"，而 nSLEEP 低时芯片进 sleep、
         nFAULT 变成高阻被上拉拉高 ⇒ 会被误读成"故障恢复了"；接着下一拍 EnsureEnabled
         又把 nSLEEP 拉高、真实故障条件仍在 ⇒ nFAULT 再拉低……
         结果是每拍在"关断→假恢复→使能→再故障"之间自激打转：驱动器几乎不出力（转子
         不动、校准照旧失败），而且**每使能一次就重新触发一次故障**，把故障"养"住。
         保持使能 + 0% 占空比（三相下桥全通 = 绕组短路 = 安全态），芯片自己的重试才有机会恢复。

       ⚠ 每拍查一次 GPIO（开销可忽略）：故障可能在校准途中才出现 */
    if (MotorPwm_FaultRead(&handle->device.pwm) != 0u)
    {
        MotorPwm_SetDuty(&handle->device.pwm, 0.0f, 0.0f, 0.0f);    /* 只清零，不动 nSLEEP */

        if (handle->drv_fault_seen == 0u) {
            handle->drv_fault_seen = 1u;            /* 边沿：记一次故障次数 */
            if (handle->drv_fault_cnt < 0xFFFFu) {
                handle->drv_fault_cnt++;
            }
        }
        if (handle->drv_fault_ticks < 0xFFFFu) {
            handle->drv_fault_ticks++;              /* 累计时长（跨多次间歇故障累加） */
        }

        if (handle->drv_fault_ticks >= DRV_FAULT_WAIT_TICKS)
        {
            /* 等够久了还不恢复：收尾（CAL 行报 driver-fault，系统停在 STOP 等人工处理） */
            EncCalib_Abort(&handle->enc_calib, ENC_FAIL_DRV);
            handle->calib_report = 1u;
            handle->mode = MODE_STOP;
        }
        return;                                     /* 故障期间不推进校准、不输出电压 */
    }

    if (handle->drv_fault_seen != 0u)
    {
        /* 故障消失：从整段开头重来（故障期间转子可能被动过，之前的相位参照已不可信） */
        handle->drv_fault_seen = 0u;
        EncCalib_Reset(&handle->enc_calib);
    }

    /* ---- 编码器数据失效（连续 >20ms 收不到有效帧）：没有位置反馈，校准不可能成功 ----
       直接中止，别白跑 6 秒再报一个误导性的 slip。
       典型根因：MISO(PC11) 断线/虚焊、SCK(PC10)/CSN(PA15) 接触不良；
       也可能是收到了恒为 0 的帧（全 0 帧会让 CRC-6 误判通过，BSP 已显式拒绝）。 */
    if (PosSensor_IsStale(&handle->device.pos) != 0u)
    {
        EncCalib_Abort(&handle->enc_calib, ENC_FAIL_ENC);
        handle->calib_report = 1u;
        MotorPwm_Stop(&handle->device.pwm);
        handle->mode = MODE_STOP;
        return;
    }

    if (EncCalib_Run(&handle->enc_calib, &g_enc_calib_cfg,
                     PosSensor_GetMechAngle(&handle->device.pos),
                     1.0f / (float)CONTROL_HIGH_HZ, &out) != 0u)
    {
        /* 校准完成：写回结果 → 关断 → 上报 → 回停止。
           结果（offset + dir）只保存在 RAM 里、每次上电重新校准（掉电保持 = 后续开发内容）；
           SUSPECT 与否在 handle->enc_calib.suspect 里，主循环打印时一并给出 */
        PosSensor_SetElecDir(&handle->device.pos, handle->enc_calib.out_dir);
        PosSensor_SetElecOffset(&handle->device.pos, handle->enc_calib.out_offset);

        handle->enc_calib_done = 1u;
        handle->calib_report   = 1u;            /* 主循环一次性打印 CAL 行（见 user.c） */
        handle->drv_fault_ticks = 0u;           /* 校准成功：驱动器故障等待预算清零 */
        handle->drv_fault_seen  = 0u;           /* （drv_fault_cnt 保留：这一轮犯过几次病要看得见） */

        MotorPwm_Stop(&handle->device.pwm);
        handle->mode = MODE_STOP;
        return;                                 /* 本拍不要再施加电压 */
    }

    MotorPwm_EnsureEnabled(&handle->device.pwm);        /* 从停止切入：一次性使能 */
    MC_SVPWM_Calc(out.ud, out.uq, out.sin, out.cos, &u, &v, &w);
    MotorPwm_SetDuty(&handle->device.pwm, u, v, w);
}

/**
 * @brief  全开环（VVVF）：无任何反馈，按给定电压幅值/电频率驱动电机旋转
 * @note   开环电角度每拍积分（θ += 2π·f / 20kHz）→ FOC SVPWM 输出；角度每拍归约回
 *         [0, 2π) 保证查表法正余弦的 float 精度（单次相减够用：步长 < 2π ⇔ f < 20kHz，
 *         远超本测试的用量）；
 *         电压幅值 target.ol_volt（调制比 0~1，1 = Udc/2），电频率 target.ol_freq (Hz)；
 *         SVPWM 相电压为马鞍波（正弦+零序分量），线电压才是正弦，属正常现象；
 *         观察波形时电频率 ≤5Hz（PRINT_PERIOD_MS=20 → 50Hz 采样，5Hz 恰好 10 点/周期），
 *         否则欠采样看不到形状
 */
static void Mode_FullOpenLoop(MC_Handle_t *handle)
{
    float u, v, w;

    handle->state.ol_angle += FOC_2PI * handle->target.ol_freq / (float)CONTROL_HIGH_HZ;
    if (handle->state.ol_angle >= FOC_2PI) {
        handle->state.ol_angle -= FOC_2PI;
    }

    MotorPwm_EnsureEnabled(&handle->device.pwm);    /* 从停止切入：一次性使能（缓存占空比生效） */

    /* 编码器校准验证观测（仅观测，不参与驱动）：
       用校准后的编码器电角度做 Clarke/Park → Id/Iq。电机同步旋转时转子磁极位置
       ≈ 开环角度；若校准正确（offset 对、方向对），Id/Iq 为平稳直线（≈直流）；
       若偏移错/方向反，Id/Iq 呈正弦波动——上位机看 ch7/ch8 即可判断校准质量 */
    {
        float s_enc, c_enc;

        Foc_GetSinCos(handle, PosSensor_GetElecSpeed(&handle->device.pos), &s_enc, &c_enc);
        Foc_ObserveDq(handle, s_enc, c_enc);
    }

    /* 开环角度的 sin/cos：同样用合并查表（省一次角度归一化） */
    {
        float s_ol, c_ol;

        MC_Math_SinCos(handle->state.ol_angle, &s_ol, &c_ol);
        MC_SVPWM_Calc(0.0f, handle->target.ol_volt, s_ol, c_ol,
                      &u, &v, &w);            /* FOC SVPWM：输出三相占空比 [0, 1]，直接写入 */
    }
    MotorPwm_SetDuty(&handle->device.pwm, u, v, w);
}

/**
 * @brief  半开环（角度闭环 + 电流开环）
 * @note   电角度取编码器反馈（含校准偏移与方向，中频环更新）→ dq 旋转坐标与转子同步；
 *         电压直接给定（无 PI）：Ud = target.ud_target（通常 0），Uq = target.uq_target
 *         （**调制比**，1 = Udc/2，非 V）；
 *         用途：验证编码器校准/电角度对齐是否正确——判据是**波动**：对齐正确则电机平稳转动、
 *         串口 Id/Iq 平稳（≈直流、无起伏）；对齐错误则电流波动/卡顿。
 *         ⚠ Id 的**均值**天然 ≈0（电压给定与电流观测用的是同一套角度）⇒ 不能用它判断对齐，
 *           详见 user.c 的 TEST_MODE 2 注释；
 *         电流开环：堵转时电流只受 R 限制（I=U/R），测试时电压给小值（≤0.3）
 */
static void Mode_HalfOpenLoop(MC_Handle_t *handle)
{
    float u, v, w, s, c;
    /* 电角速度（rad/s）：位置传感器已按 机械 rps × 2π × 极对数 算好，供角度延迟补偿用 */
    float we = PosSensor_GetElecSpeed(&handle->device.pos);

    Foc_GetSinCos(handle, we, &s, &c);      /* 含角度延迟前推补偿 */

    MotorPwm_EnsureEnabled(&handle->device.pwm);    /* 从停止切入：一次性使能 */

    Foc_ObserveDq(handle, s, c);        /* 仅观测 Id/Iq 反馈，不闭环 */

    handle->state.ud = handle->target.ud_target;
    handle->state.uq = handle->target.uq_target;

    /* 注意单位：ud_target / uq_target 是调制比（1 = Udc/2），不是 V——
       要和"安培"的 id_target/iq_target 分清（半开环只给电压、不给电流） */
    MC_SVPWM_Calc(handle->target.ud_target, handle->target.uq_target, s, c, &u, &v, &w);
    MotorPwm_SetDuty(&handle->device.pwm, u, v, w);
}

/**
 * @brief  电流环闭环（角度闭环 + 电流闭环）
 * @note   编码器电角度 → Clarke/Park → dq 电流环（mc_current：PI + 前馈解耦 +
 *         矢量限幅 + 抗饱和）→ Ud/Uq → 逆 Park + SVPWM；给定 iq_target（A）后
 *         持续产生转矩 → 电机旋转（转速随电压可用量自由上升）；
 *         启动前提：电流校准 + 编码器校准已完成（就绪判断在 user 测试块）；
 *         电压限幅 CURRENT_U_MAX（矢量幅值，调制比）内自动避免过调制；
 *         带软件过流保护：连续超 CURRENT_I_TRIP 即关断并锁定（见下）
 */
static void Mode_CurrentLoop(MC_Handle_t *handle)
{
    float u, v, w;
    float s, c;
    float ud, uq, i2;

    /* 电角速度（rad/s，电机坐标系：正 = 电角度增大方向）：位置传感器按低通机械转速
       （中频环 10kHz 解算，fc≈95Hz）× 2π × 极对数给出；前馈与角度延迟补偿都用它 */
    float we = PosSensor_GetElecSpeed(&handle->device.pos);

    /* ---- 编码器新鲜度看门狗（安全兜底）----
       数据冻结时（连续 >20ms 没有新的有效帧，见 PosSensor_IsStale）：角度不再更新
       ⇒ 磁场不会旋转 ⇒ **有电流、没转矩**（这就是"电机不转但一直有功率"的机理）。
       宁可不出力：把给定归零（编码器一恢复就自动接着跑），至少不让电流白灌、不发烫。
       ⚠ 这是**事后兜底**：真正该查的是链路本身（SSI 帧为何断），诊断信息见
         CAL 行的 emax= 与 user.c 里那条一次性 FAULT: encoder stale */
    if (PosSensor_IsStale(&handle->device.pos) != 0u)
    {
        handle->state.id_ref = 0.0f;
        handle->state.iq_ref = 0.0f;
    }

    Foc_GetSinCos(handle, we, &s, &c);      /* 含角度延迟前推补偿 */

    /* Clarke + Park（编码器电角度）→ state.id / state.iq 反馈 */
    Foc_ObserveDq(handle, s, c);

    /* 软件过流保护：电流矢量幅值连续超阈值 → 关断 + 锁定（测试入口不再自动切入）。
       额定 0.5A、阈值 1.2A、连续 3 拍（0.15ms）触发。为什么需要：电流反馈符号若接反，
       闭环变正反馈，电流会顶到电压极限堵转值（1.10×6V/3.5Ω ≈ 1.9A），
       硬件 OCP 只保驱动器、这里保电机。用平方比较避免开方（工程无 libm） */
    i2 = handle->state.id * handle->state.id + handle->state.iq * handle->state.iq;
    if (i2 > (CURRENT_I_TRIP * CURRENT_I_TRIP))
    {
        if (++handle->oc_cnt >= CURRENT_I_TRIP_N)
        {
            handle->oc_cnt       = 0u;
            handle->oc_latched   = 1u;
            handle->fault_report = 1u;
            MotorPwm_Stop(&handle->device.pwm);     /* 关断输出（并锁定：测试入口不再自动切入） */
            CurrentLoop_Reset(&handle->current);
            handle->mode = MODE_STOP;
            return;
        }
    }
    else
    {
        handle->oc_cnt = 0u;
    }

    /* dq 电流环（给定 id_ref/iq_ref，单位 A；输出 Ud/Uq，单位调制比）。
       给定由**当前模式**写在 handle->state.id_ref / iq_ref 里（见本文件高频环分发）：
         电流环模式   = target.id_target / target.iq_target（用户给定）
         速度/位置模式 = 速度环输出（中频环每拍写）
       进入本函数前，任何来源的给定都已逐轴钳到 ±CURRENT_I_MAX_GUARD（在 CurrentLoop_Run 内） */
    CurrentLoop_Run(&handle->current, 1.0f / (float)CONTROL_HIGH_HZ,
                    handle->state.id_ref, handle->state.iq_ref,
                    handle->state.id, handle->state.iq, we,
                    &ud, &uq);
    handle->state.ud = ud;
    handle->state.uq = uq;

    /* 从停止切入：驱动器先前关断 → 清电流环/速度环积分（避免带上一次运行的残留）
       并一次性使能。MotorPwm_EnsureEnabled 返回 1 表示"本次刚使能"，正好对应这个一次性动作；
       高频环先于中频环执行 → 速度环积分/斜坡也在它本拍运行前就清好了 */
    if (MotorPwm_EnsureEnabled(&handle->device.pwm) != 0u) {
        CurrentLoop_Reset(&handle->current);
        SpeedLoop_Reset(&handle->speed);
    }

    MC_SVPWM_Calc(ud, uq, s, c, &u, &v, &w);
    MotorPwm_SetDuty(&handle->device.pwm, u, v, w);
}

/* ---- 速度环（中频环 10kHz 执行，不是高频环每拍；见第 4 节 Control_MidLoop） ---- */

/**
 * @brief  速度电流环（外环速度环 + 内环电流环，级联）
 * @note   分工：本函数只做"速度 → 电流给定"（中频环 10kHz），
 *         电流环本体仍由高频环的 Mode_CurrentLoop 执行（20kHz）——
 *         Park/SVPWM/电压限幅/过流保护全部复用，没有任何重复实现。
 *         输出写 handle->state.iq_ref（外环输出 → 内环给定），**不动 target**：
 *         target 只放用户给定，state 放"当前真正生效的给定"，两者不再混淆；
 *         id_ref 恒 0（表贴式 PMSM，不弱磁、不做 MTPA）。
 *         两环频率比 20kHz:10kHz=2 已足够（速度环带宽 10Hz，电流环 400Hz）。
 *         反馈用 PosSensor_GetSpeedRpm（已低通、电机坐标系）；切入清零见 Mode_CurrentLoop
 */
static void Mode_SpeedCurrentLoop(MC_Handle_t *handle)
{
    float iq = SpeedLoop_Run(&handle->speed,
                             1.0f / (float)CONTROL_MID_HZ,
                             handle->target.spd_target,
                             PosSensor_GetSpeedRpm(&handle->device.pos),
                             handle->target.iq_max);     /* 用户限幅（速度环内再与保底限幅取小） */

    handle->state.id_ref = 0.0f;    /* d 轴给定恒 0：表贴式无磁阻转矩，不需要 d 轴电流 */
    handle->state.iq_ref = iq;      /* 外环输出 → 内环给定（高频环下一拍读走） */
}

/* ==================== 4. 对外接口 ==================== */

/**
 * @brief  初始化：设备初始化 + 校准状态复位 + PI 参数
 * @note   上电默认进入电流校准模式，之后自动进入编码器校准（或跳过）→ 停止
 */
void Control_Init(MC_Handle_t *handle)
{
    /* 设备初始化（写入 handle->device） */
    MotorPwm_Init(&handle->device.pwm);             /* 启动三路载波，0% 占空比 */
    PosSensor_Init(&handle->device.pos, POS_SENSOR_SSI, MOTOR_POLE_PAIRS);   /* 位置传感器（SSI） */
    CurrentSensor_Init(&handle->device.cur, CURRENT_SENSOR_3PH);             /* ADC 校准 + CC4 触发 + 注入转换 */

    handle->sched.mid_cnt = 0u;
    handle->sched.low_cnt = 0u;

    /* 编码器校准状态机：复位到整段最开头（进入校准模式即从锁轴/升速开始） */
    EncCalib_Reset(&handle->enc_calib);

    /* 编码器校准偏移 + 方向：当前每次上电都重新执行编码器校准，
       不读非易失存储恢复、不落盘（掉电保持 = 后续开发内容）。
       后续要恢复时，在此处读出 offset/dir 后：
           方向必须先于偏移恢复（PosSensor_SetElecDir → PosSensor_SetElecOffset），
           两者必须成对使用（同一次校准的产物），恢复成功再把 enc_calib_done 置 1 */
    handle->enc_calib_done = 0u;    /* 强制上电执行编码器校准 */
    handle->calib_report   = 0u;
    handle->oc_latched     = 0u;    /* 过流锁定：上电清零 */
    handle->fault_report   = 0u;
    handle->oc_cnt         = 0u;
    handle->drv_fault_ticks = 0u;   /* 驱动器故障计时/计数：上电清零 */
    handle->drv_fault_cnt   = 0u;
    handle->drv_fault_seen  = 0u;

    /* 电流环：增益由电气参数 + 目标带宽算（推导见 control.h / mc_current.h），
       量出真实 R/L 后改宏即可；运行中也能直接改 mc.current.kp/ki 现场试 */
    {
        float wc     = FOC_2PI * CURRENT_BW_HZ;          /* 目标带宽（rad/s） */
        float u_half = 0.5f * MOTOR_UDC_NOMINAL;         /* 调制比 1.0 对应的相电压峰值 (V) */
        float kp     = (MOTOR_LD * wc) / u_half;         /* 调制比/A */
        float ki     = (MOTOR_R_PHASE * wc) / u_half;    /* 调制比/(A·s) */

        CurrentLoop_Init(&handle->current, kp, ki, CURRENT_U_MAX, CURRENT_I_MAX_GUARD,
                         1.0f / u_half);                 /* v2m = 2/Udc：伏特→调制比 */
                                                         /* i_max：保底限幅（内环逐轴钳位，
                                                            任何来源的给定都顶不破） */
        CurrentLoop_SetFeedforward(&handle->current, MOTOR_LD, MOTOR_LQ,
                                   MOTOR_FLUX, (uint8_t)CURRENT_FF_EN);
    }

    /* 速度环：增益同样由"目标带宽 + 机电参数"算（推导见 control.h / mc_speed.h），
       J 已按实测反推值填入（MOTOR_INERTIA = 3e-6，见 control.h）
           Kt = 1.5×极对数×ψf [N·m/A]（1.5 来自幅值不变 Clarke/Park）
           kp = J·ωsc/Kt [A/(rad/s)] → 乘 (2π/60) 换成 A/rpm（误差/给定都用 rpm）
           ki = kp·ωsc×SPEED_KI_RATIO [A/(rpm·s)]：PI 零点放 ωsc/4（相位裕度 ≈76°），
              该比例可调（置 0 = 纯比例，用于判断振荡是 ki 还是 kp 引起） */
    {
        float kt     = 1.5f * (float)MOTOR_POLE_PAIRS * MOTOR_FLUX;   /* 转矩常数 (N·m/A) */
        float wsc    = FOC_2PI * SPEED_BW_HZ;                         /* 目标带宽 (rad/s) */
        float rpm2rs = FOC_2PI * (1.0f / 60.0f);                      /* rpm → rad/s */
        float kp     = (MOTOR_INERTIA * wsc / kt) * rpm2rs;           /* A/rpm */
        float ki     = kp * wsc * SPEED_KI_RATIO;                     /* A/(rpm·s) */

        SpeedLoop_Init(&handle->speed, kp, ki,
                       CURRENT_I_MAX_USER, CURRENT_I_MAX_GUARD, SPEED_REF_RAMP_RPM_S);
    }

    /* 用户限幅（速度/位置模式的转矩限幅）：上电默认 = 额定；
       之后用 Control_SetCurrentLimit() 调整（保底限幅 CURRENT_I_MAX_GUARD 不可改） */
    Control_SetCurrentLimit(handle, CURRENT_I_MAX_USER);

    handle->mode = MODE_CURRENT_CALIB;
}

/**
 * @brief  设置用户限幅（速度/位置模式的转矩限幅）
 * @note   只写 target.iq_max：速度环每拍按 min(用户限幅, 保底限幅) 限幅 →
 *         改完立即生效；实际生效值可在 mc.speed.iq_max 里读到（每拍刷新）。
 *         保底限幅（CURRENT_I_MAX_GUARD）在这里钳一次，mc_speed 里再 min() 一次，
 *         电流环还对任何给定做一次逐轴钳位 —— 三道都在，绕过任何一道都顶不破
 */
void Control_SetCurrentLimit(MC_Handle_t *handle, float a)
{
    if (a < 0.0f)
    {
        a = 0.0f;
    }
    if (a > CURRENT_I_MAX_GUARD)
    {
        a = CURRENT_I_MAX_GUARD;    /* 用户限幅最多只能要到保底限幅 */
    }

    handle->target.iq_max = a;
}

/**
 * @brief  分频调度器（在 20kHz ADC 注入中断中调用）
 */
void Control_Sched(MC_Handle_t *handle)
{
    /* 分频调度：高频每拍；中频每 CONTROL_MID_DIV 拍（2 分频 = 10kHz）；
       低频每 CONTROL_LOW_DIV 拍（4 分频 = 5kHz） */
    Control_HighLoop(handle);

    if (++handle->sched.mid_cnt >= CONTROL_MID_DIV) {
        handle->sched.mid_cnt = 0u;
        Control_MidLoop(handle);
    }
    if (++handle->sched.low_cnt >= CONTROL_LOW_DIV) {
        handle->sched.low_cnt = 0u;
        Control_LowLoop(handle);
    }
}

/**
 * @brief 高频环（20kHz，每拍）：传感器更新 + 按模式分发到各 Mode_xxx
 */
void Control_HighLoop(MC_Handle_t *handle)
{
    /* ---- 传感器信息更新（switch 外，任何模式都运行） ---- */
    CurrentSensor_Update(&handle->device.cur);      /* 一直更新：每拍换算三相电流 */

    /* ---- 模式分发：具体实现在本文件第 3 节 ---- */
    switch (handle->mode)
    {
    case MODE_STOP:                   Mode_Stop(handle);         break;
    case MODE_CURRENT_CALIB:          Mode_CurrentCalib(handle); break;
    case MODE_ENCODER_CALIB:          Mode_EncoderCalib(handle); break;
    case MODE_FULL_OPEN_LOOP:         Mode_FullOpenLoop(handle); break;
    case MODE_HALF_OPEN_LOOP:         Mode_HalfOpenLoop(handle); break;
    /* 电流环本体三种模式完全相同，区别只在"给定从哪来"：
         电流环模式   —— 每拍把用户给定搬到 state.id_ref/iq_ref（内环只认 state）
         速度/位置模式 —— state.id_ref/iq_ref 由中频环的外环写好（id_ref 恒 0） */
    case MODE_CURRENT_LOOP:
        handle->state.id_ref = handle->target.id_target;
        handle->state.iq_ref = handle->target.iq_target;
        Mode_CurrentLoop(handle);
        break;
    case MODE_SPEED_CURRENT_LOOP:
    case MODE_POS_SPEED_CURRENT_LOOP:
        Mode_CurrentLoop(handle);
        break;
    default:                          break;
    }
}

/**
 * @brief 中频环（10kHz，2 分频）
 * @note  位置/角度/速度更新在 switch 外（任何模式都运行）；
 *        再按模式分发：速度环（MODE_SPEED_CURRENT_LOOP / POS_SPEED_CURRENT_LOOP）
 */
void Control_MidLoop(MC_Handle_t *handle)
{
    /* ---- 传感器信息更新（switch 外，任何模式都运行） ---- */
    PosSensor_Update(&handle->device.pos, CONTROL_MID_HZ);  /* 位置/角度/速度 */

    /* ---- 模式分发 ---- */
    switch (handle->mode)
    {
    /* 速度环：spd_target(rpm) + 用户限幅 iq_max(A) → state.iq_ref(A)。
       位置模式下 spd_target 由低频环的位置环写入，速度环照跑（本环在 10kHz） */
    case MODE_SPEED_CURRENT_LOOP:
    case MODE_POS_SPEED_CURRENT_LOOP:
        Mode_SpeedCurrentLoop(handle);
        break;
    default:
        break;
    }
}

/**
 * @brief 低频环（5kHz，4 分频）
 * @note  按模式分发（位置环待实现：pos_target → spd_target，速度环在中频环执行）
 */
void Control_LowLoop(MC_Handle_t *handle)
{
    switch (handle->mode)
    {
    case MODE_POS_SPEED_CURRENT_LOOP:
        /* TODO: 位置环（handle->target.pos_target → handle->target.spd_target） */
        break;

    default:
        break;
    }
}
