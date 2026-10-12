/**
 * @file    mc_enc_calib.h
 * @brief   编码器校准算法：两点锁轴 / 慢速旋转（平台无关，不碰设备与数据库）
 *
 * 为什么单独成模块（与 mc_current / mc_speed 同理）：
 *   · 这里是**纯算法 + 状态机**：输入"机械角 + 本拍 dt"，输出"本拍该施加的 dq 电压矢量"
 *     与"完成时的 dir / offset"。传感器读取、PWM 输出、关断、结果落库全留在 Control 层；
 *   · Control 层因此只剩"读角度 → 调算法 → 写设备"的胶水（约 20 行），模式分发保持易读；
 *   · 本模块**不依赖 control.h**：可调参数由调用方用 EncCalib_Cfg_t 传进来
 *     （时间类参数在 Control 层就折成拍数），所以能脱离硬件单独做数值验证。
 *
 * 两种方式（由 cfg.method 选，常量在本文件定义、control.h 的 ENC_CALIB_METHOD 引用它们）：
 *
 *   方式一 两点锁轴（ENC_CALIB_METHOD_2POINT）
 *     把电压矢量锁在两个已知电角度，各等振动收敛后平均机械角：
 *       点 0：矢量锁电角度 0°（Uα=U，A 相磁轴）→ 定零点 0 = dir·pp·θmech0 + offset
 *       点 1：矢量锁电角度 +90°（Uβ=U）     → 定方向 π/2 = dir·pp·θmech1 + offset
 *     ⇒ dir = sign(θmech1 − θmech0)、offset = NormRad(−dir·pp·θmech0)
 *     快。短板：只采到**两个**转子位置，那两处"电磁对齐转矩 vs 齿槽/静摩擦"的平衡误差
 *     **整份**进 offset，且每次上电落点随机 ⇒ 零位残余 3~5° 电角度。
 *
 *   方式二 慢速旋转（ENC_CALIB_METHOD_ROT，推荐/默认）
 *     按电频率拖一个旋转电压矢量（开环同步拖转，像步进电机软启动），三个状态循环两遍：
 *       ALIGN  先把频率**软降到 0**（上电时入口频率就是 0，等于静态吸附），再用
 *              "最近 W 拍位置没动"判据等到转子**真的静止**才往下走 —— 这一步是稳的关键：
 *              上电时转子停在任意位置、被拉到对齐点会有一次冲击振荡（本机磁井谐振约 31.5Hz、
 *              阻尼很小），若不等它衰减就启转，很容易丢步（而且丢步与初始位置有关 ⇒ 随机失败）
 *       PRE    频率从 0 线性升到 ±f，再保持 SETTLE 时间（只跟踪、不测量）
 *       MEAS   以 ±f 拖转，累加 e = θe_cmd − dir·pp·θm，走够 REVS 个机械圈
 *     正向测完 → 回到 ALIGN 把 +f 软停到 0 并等静止 → PRE 反向升速 → 反向测量。
 *     ⚠ 约定（最隐蔽的坑，第一版就错在这）：电压矢量必须压在**指令 d 轴**上
 *       （ud = rot_volt、uq = 0）。因为 MC_SVPWM_Calc 的 (s,c) 描述的是转子 d 轴，
 *       而转子 d 轴会追着电流矢量跑；若用 (0, V)（即在指令 q 轴上），转子 d 轴实际
 *       会停在 θ_cmd+90° 处 ⇒ 算出的 offset 整整小 90° ⇒ 闭环时电流全落在真 d 轴上
 *       ⇒ "CAL 行一切正常，但电机不转、一直吃电流"。详见 mc_enc_calib.c 的输出注释
 *     为什么更准：
 *       ① 齿槽误差是"转子位置"的周期函数（12槽14极 → 齿槽周期 30° 电角度），沿整圈
 *          （1 机械圈 = 84 个齿槽周期）取平均后只剩高阶残余；两点锁轴只采两个位置，
 *          那两处的误差整份进 offset ⇒ 3~5° 随机。
 *       ② 转子必须落后磁场 δe 才出得来克服摩擦的转矩，单向测得的均值里混着 δe；
 *          正反两向的 δe 符号相反：e_fwd = offset + δe、e_rev = offset − δe
 *          ⇒ **圆平均**即得 offset，差值的一半 δe = (e_fwd − e_rev)/2 还被顺手读出来。
 *
 *   ⚠ 失步的判据（最容易踩的坑）：**丢一步 = 360° 电角度 = 测量窗的 14.3%**，
 *     所以跟随性比值 ratio = |Δθm|·pp/|Δθe| 一旦偏离 1，offset 的偏差 ≈ 滑移量的一半
 *     （丢一步就是 ~180° 电角度 ⇒ 结果彻底不可用）。⇒ ratio 的接受窗口必须远小于 14.3%
 *     （当前 ±2%），并配合**自动重试**：丢步是暂态的，重来一次通常就过了。
 *
 *   ⚠ 另一个坑（已修，值得记）：每段 ALIGN 会按当前机械角**重新播种**解绕量，这会让
 *     e = θe_cmd − dir·pp·θm 整体平移 dir·pp·2π·m（是 2π 的整数倍 ⇒ e mod 2π 不变，
 *     但**绝对值能差好几整圈**：本机正段正好转 1 机械圈 ⇒ m≈1 ⇒ 差 7 圈）。
 *     所以两个方向的估计**必须先各自归一到 [0,2π) 再算圆平均**：
 *       · 不归一：off_fwd − off_rev 差好几圈，dd/2 的误差是 π·(整数) —— **K 为偶数时
 *         offset 真偏 180°，K 为奇数时又侥幸正确**，而 K 取决于上电时的随机落点
 *         ⇒ 表现为"有时行有时不行"，极难查；
 *       · 归一后 |dd| < 2π，一次解绕即得 dd = 2δe，offset 与 δe 都干净。
 *     ✓实测（同一固件两次上电）：off 相差 0.06° 电角度（两点锁轴是 3~5° 随机）。
 *
 *   ⚠ 再一个坑（已修，第一版就错在这，且最隐蔽）：拖转矢量的**轴**必须与标定数学一致。
 *     MC_SVPWM_Calc 的 (s,c) 描述的是**转子 d 轴**，而转子 d 轴会追着电流矢量跑，
 *     所以矢量必须压在**指令 d 轴**上（ud = rot_volt、uq = 0）：
 *       用 (0, V) ⇒ αβ 矢量在 θ_cmd+90° ⇒ 转子 d 轴停在 θ_cmd+90°
 *                 ⇒ 算出的 offset 整整小 90°
 *                 ⇒ 闭环时电流全落在**真 d 轴**上 ⇒ **电机不转但一直吃电流**，
 *                   而 CAL 行（ratio=1.000、load=δe、dir 都对）看起来**完全正常**
 *                 ⇒ 没有任何判据能发现它，只能靠"两法交叉核对 off"或直接看电机转不转。
 *     ✓修复验证：off 从 4.9973 跳到 0.2853 rad —— 差值 270.02° = −90°，
 *       与修复前预测的 0.2848 rad 相差仅 0.03°；修复后电机正常转动。
 */
#ifndef __MC_ENC_CALIB_H__
#define __MC_ENC_CALIB_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 校准方式常量（control.h 的 ENC_CALIB_METHOD 用这两个值） ==================== */
#define ENC_CALIB_METHOD_2POINT   0     /**< 两点锁轴：快，零位残余 3~5° 随机 */
#define ENC_CALIB_METHOD_ROT      1     /**< 慢速旋转：慢，齿槽与负载角都被消掉 */

/* ==================== 失败原因（h->rot_fail 位掩码，用 EncCalib_FailStr() 取字符串） ==================== */
#define ENC_FAIL_ALIGN    0x01u     /**< 对齐段超时：软停后转子一直没静止（机械/编码器问题） */
#define ENC_FAIL_MEAS_TO  0x02u     /**< 测量段超时：没走够要求的圈数（没转起来/编码器不更新） */
#define ENC_FAIL_SLIP     0x04u     /**< 跟随性比值越界：丢步（净滑移）⇒ offset 不可信 */
#define ENC_FAIL_LOAD     0x08u     /**< 跟随负载角越界：没同步/摩擦过大 */
#define ENC_FAIL_DRV      0x10u     /**< 外部中止：驱动器故障（nFAULT 拉低，输出级已关断）
                                         ⇒ 没电流就没转矩，校准不可能成功，不必白跑
                                         （见 EncCalib_Abort） */
#define ENC_FAIL_ENC      0x20u     /**< 外部中止：编码器数据失效（连续 >20ms 收不到有效帧）
                                         ⇒ 没有位置反馈，校准不可能成功（见 EncCalib_Abort）。
                                         常见根因：MISO(PC11) 断线/虚焊、SCK(PC10)、CSN(PA15) */

/* ==================== 方式一：两点锁轴 阶段 ==================== */
typedef enum {
    ENC_CALIB_PHASE_LOCK   = 0,   /**< 锁轴：固定矢量锁转子（点 0 = 电角度 0°、点 1 = +90°），等振动收敛 */
    ENC_CALIB_PHASE_READ   = 1,   /**< 读数：位置稳定后多拍平均机械角 → 点 0 定零点 / 点 1 定方向 */
} EncCalib2pPhase_t;

/* ==================== 方式二：慢速旋转 阶段 ==================== */
typedef enum {
    ENC_ROT_ALIGN = 0,      /**< 对齐：频率软降到 0（上电时本就是 0），再等转子真的静止 */
    ENC_ROT_PRE   = 1,      /**< 升速：0 → ±f 线性，再保持 SETTLE（只跟踪，不测量） */
    ENC_ROT_MEAS  = 2,      /**< 测量：以 ±f 拖转，累加 e 与行程，走够 REVS 圈 */
} EncCalibRotPhase_t;

/* ==================== 参数（调用方从自己的宏折好传进来） ====================
 * 本模块不认识 control.h 的宏名，字段全是"算法要用的形式"：
 * 时间都在 Control 层折成拍数（省掉运行期乘除），角度阈值统一用 rad。
 */
typedef struct {
    uint8_t  method;         /**< 校准方式：ENC_CALIB_METHOD_2POINT / _ROT */
    int8_t   dir_force;      /**< 方向强制：0 = 自动判；±1 = 强制该方向（调试，用来 A/B 验证判据） */
    uint16_t pole_pairs;     /**< 极对数 pp（= MOTOR_POLE_PAIRS） */

    /* ---- 方式一 两点锁轴 ---- */
    float    lock_volt;      /**< 锁轴电压（调制比） */
    uint16_t min_settle;     /**< 最小锁轴等待（拍）：先让它收敛再判稳定 */
    uint16_t max_settle;     /**< 最长锁轴等待（拍）：超时强制进读数兜底 */
    uint16_t read_n;         /**< 读数平均拍数 */
    float    stable_err;     /**< 稳定判据：机械角波动上限（rad） */
    float    dir_min_ratio;  /**< 方向判据接受窗口（相对理论位移 π/(2·pp)） */
    float    dir_max_ratio;

    /* ---- 方式二 慢速旋转 ---- */
    float    rot_volt;       /**< 旋转矢量幅值（调制比）：要留足"顶过齿槽"的余量 */
    float    rot_freq;       /**< 电频率（Hz） */
    uint16_t rot_ramp_ticks;      /**< 升/降速拍数（软启动、软停车、软换向） */
    uint16_t rot_settle_ticks;    /**< 升速后的稳定拍数（PRE 段，只跟踪不测量） */
    uint16_t rot_align_min_ticks; /**< 对齐段最小等待：软停完成后至少这么多拍才判"静止" */
    uint16_t rot_align_max_ticks; /**< 对齐段最长等待：超时则带 ENC_FAIL_ALIGN 继续 */
    uint16_t rot_win_ticks;       /**< "静止"判据的滑动窗长度：与 W 拍前的角比较 */
    uint16_t rot_max_meas_ticks;  /**< 测量段超时拍数（到点没走够 ⇒ ENC_FAIL_MEAS_TO） */
    float    rot_travel_target;   /**< 每个测量段要走够的电角度（rad）= 2π·pp·REVS */
    float    rot_ratio_min;       /**< 跟随性比值 |Δθm|·pp/|Δθe| 的接受窗口：
                                       **必须远小于一次丢步的 14.3%**（当前 ±2%）。
                                       它同时复核极对数：pp 填错会按比例偏离 1 */
    float    rot_ratio_max;
    float    rot_max_load;        /**< 跟随负载角 δe 的上限（rad 电角度） */
    uint8_t  rot_retry_n;         /**< 判据不过时自动重来几次（0 = 不重试）；丢步是暂态的，重试很有效 */
} EncCalib_Cfg_t;

/* ==================== 句柄（运行状态 + 诊断量） ==================== */
typedef struct {
    /* ---- 方式一 两点锁轴 ---- */
    EncCalib2pPhase_t phase;  /**< 当前阶段 */
    uint16_t tick;            /**< LOCK：已运行拍数 */
    uint16_t n;               /**< READ：已平均拍数 */
    float    first;           /**< LOCK：**解绕后**机械角最小值（波动检测下限） */
    float    last;            /**< LOCK：**解绕后**机械角最大值（波动检测上限） */
    float    sum;             /**< READ：**解绕后**机械角累加（取平均后再归一回 [0,2π)） */
    float    mech0;           /**< 点 0（电角度 0°）的平均机械角 θmech0 (rad) */
    float    dmech;           /**< 两点机械角差 (rad)：名义 ±π/(2·pp)，校准诊断用 */
    uint8_t  point;           /**< 锁轴点：0 = 电角度 0°（定零点），1 = +90°（定方向） */

    /* ---- 两种方式共用 ---- */
    float    unwrapped;       /**< 解绕后的连续机械角 (rad)：编码器原始角归一在 [0,2π)，
                                   跨 0/2π 会跳变 2π，而两种方式要的都是"连续量之差" */
    uint8_t  suspect;         /**< 1 = 结果可疑（方式一：第二点没动/跳远；方式二：见 rot_fail） */
    uint8_t  done;            /**< 1 = 已完成：Run 幂等地返回结果，不再推进状态 */
    int8_t   out_dir;         /**< 完成时的方向（±1）：写回 PosSensor_SetElecDir */
    float    out_offset;      /**< 完成时的电角度偏移 (rad，[0,2π))：写回 PosSensor_SetElecOffset */

    /* ---- 方式二 慢速旋转 ---- */
    EncCalibRotPhase_t rot_phase; /**< 当前阶段 */
    uint8_t  rot_stage;       /**< 0 = 正在测正向；1 = 正在测反向 */
    uint8_t  rot_attempt;     /**< 第几次尝试（0 = 第一次；重试后递增，报告里可见） */
    uint8_t  rot_fail;        /**< **本次**尝试的失败原因位掩码（ENC_FAIL_*，0 = ok） */
    uint8_t  rot_fail_first;  /**< **第一次**尝试的失败原因：重试成功时 rot_fail 会被清零，
                                   不单独留一份的话"当初为什么重试"就永远查不到了 */
    int8_t   rot_dir;         /**< 旋转中判出的方向（±1）：由正向拖转的机械净位移符号定 */
    float    rot_f_entry;     /**< ALIGN 段的入口频率（Hz）：软停车的起点
                                   （上电时 0；换向时 ±f；重试时 ∓f） */
    uint16_t rot_tick;        /**< 当前阶段已运行拍数（每段入口清零） */
    uint16_t rot_n;           /**< 本测量段累加拍数 */
    float    rot_ang;         /**< 电角度指令**连续量** (rad)：e = rot_ang − dir·pp·θm。
                                   ⚠ 全程不清零：它与 dir·pp·θm 同步增长，差值才是 offset */
    float    rot_ang_norm;    /**< 电角度指令归一 [0,2π)：只给查表正余弦用 */
    float    rot_travel;      /**< 本测量段已走的电角度（带符号，rad）：够 rot_travel_target 就收尾 */
    float    rot_sum_e;       /**< Σe：测量段均值 = offset（正向还含 +δe、反向含 −δe） */
    float    rot_e_min;       /**< e 的最小/最大值：峰峰值 = 齿槽+噪声造成的偏差幅度 */
    float    rot_e_max;       /**< （平均值才是零位误差；沿整圈平均后只剩高阶残余）
                                   ⚠ 正向段测完就把差值存进 rot_resid —— 因为反向段的 PRE
                                     会重置这两个值，直接打印它们会得到初始化值 */
    float    rot_resid;       /**< 正向段的残差峰峰值 (rad 电角度)：报告用（已归一） */
    float    rot_win_ref;     /**< 静止判据的滑动窗参照（W 拍前的机械角） */
    uint16_t rot_win_cnt;     /**< 滑动窗计数（到 0 时判一次并刷新参照） */
    float    rot_mech;        /**< 解绕机械角参照点：定 dir（PRE 段）与算跟随性比值（MEAS 段）用 */
    float    rot_off_fwd;     /**< 正向测得的 offset 估计（= 真值 + δe） */
    float    rot_off_rev;     /**< 反向测得的 offset 估计（= 真值 − δe） */
    float    rot_ratio_fwd;   /**< 正向跟随性比值（单独留着，便于看出是哪一段丢的步） */
    float    rot_ratio_rev;   /**< 反向跟随性比值 */
    float    rot_ratio;       /**< 两方向里较差的比值（判据用的就是这个） */
    float    rot_load;        /**< 跟随负载角 δe (rad 电角度）=(off_fwd − off_rev)/2，观测量 */
} EncCalib_t;

/* ==================== 每拍输出 ==================== */
typedef struct {
    float   ud, uq;     /**< 本拍要施加的 dq 电压给定（调制比）：调用方送 MC_SVPWM_Calc */
    float   sin, cos;   /**< 本拍坐标系：方式一恒为 (0,1)（矢量固定在 α 轴或 β 轴）；
                             方式二为当前电角度（矢量在旋转） */
    uint8_t done;       /**< 1 = 校准完成：调用方关断输出、把 out_dir/out_offset 写回传感器、
                             切回 MODE_STOP（本拍不要再施加电压） */
} EncCalib_Out_t;

/**
 * @brief  复位：回到"整段最开头"，清掉两种方式的全部过程量与诊断量
 * @param  h  句柄
 * @note   进入校准模式前调用（control.c 的 Mode_CurrentCalib 里）。
 *         复位后 dir=+1、offset=0 只是占位，真正的结果要等 done=1
 */
void EncCalib_Reset(EncCalib_t *h);

/**
 * @brief  执行一拍编码器校准（20kHz 高频环内每拍调用；纯计算，不碰设备）
 * @param  h     句柄
 * @param  cfg   参数（method 决定跑哪一种；其余字段按方式取用）
 * @param  mech  当前**原始**机械角 (rad)：PosSensor_GetMechAngle()（未解绕，本函数内部解绕）
 * @param  dt    本拍时间步长 (s) = 1/调用频率（20kHz → 5e-5）
 * @param  out   输出：本拍电压矢量 / 完成标志（见 EncCalib_Out_t）
 * @retval 1 = 校准完成（结果在 h->out_dir / h->out_offset，out->done 也为 1）；0 = 进行中
 * @note   完成之后**幂等**：再次调用直接返回 1 与同一结果，不会重复推进状态
 */
uint8_t EncCalib_Run(EncCalib_t *h, const EncCalib_Cfg_t *cfg,
                     float mech, float dt, EncCalib_Out_t *out);

/**
 * @brief  外部中止校准（例如驱动器报故障、输出级被关断）
 * @param  h     句柄
 * @param  fail  失败原因位（通常 ENC_FAIL_DRV）
 * @note   用途：调用方发现"根本不可能成功"的外部条件时（典型：DRV8311 nFAULT 拉低
 *         ⇒ 没电流就没转矩 ⇒ 转子纹丝不动、还会被误判成丢步、白跑两次重试），
 *         立刻把它标成可疑并置 done，避免继续空转。
 *         ⚠ out_dir/out_offset 不会被赋值成有效结果：调用方**不得**把结果写回传感器
 *         （control.c 的中止路径在本函数后直接 return）
 */
void EncCalib_Abort(EncCalib_t *h, uint8_t fail);

/**
 * @brief  取"结果可疑"的可读原因（给 CAL 行打印用）
 * @param  h  句柄
 * @retval 字符串常量；h->suspect == 0 时返回 "ok"
 * @note   多个原因同时成立时按"最能指明排查方向"的优先级返回一个：
 *         align（没静止）> 测量超时（没转起来）> slip（丢步）> load（没同步）
 */
const char *EncCalib_FailStr(const EncCalib_t *h);

/**
 * @brief  把失败原因位掩码折成一个短代号（给 CAL 行打印用）
 * @param  fail  位掩码（ENC_FAIL_*）
 * @retval "align" / "no-run" / "slip" / "load"；无标志时 "-"
 * @note   优先级同 EncCalib_FailStr：align > no-run > slip > load
 */
const char *EncCalib_FailCode(uint8_t fail);

#ifdef __cplusplus
}
#endif

#endif /* __MC_ENC_CALIB_H__ */
