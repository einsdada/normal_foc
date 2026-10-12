#include "user.h"
#include "user_debug.h"
#include "led.h"
#include "tim.h"
#include "usart.h"
#include "mc_db.h"
#include "control.h"
#include "pos_sensor.h"
#include "current_sensor.h"

#define TICK_FREQ_HZ      1000     /* TIM6 中断频率（Hz）= 1ms 节拍：用作 LED 状态机更新率
                                       （Led_Update 的入参，见 User_Loop）。
                                       ⚠ 这不是位置传感器的更新率——位置/角度/速度在中频环
                                       10kHz 更新（CONTROL_MID_HZ，见控制层） */
#define PRINT_PERIOD_MS    20       /* 串口打印周期（ms）：16 通道 CSV 一行最坏约 113 字节
                                       （16 个数 + 15 个逗号 + 换行），115200 8N1 下发一行要
                                       ~9.8ms → 周期 10ms 会把串口占满、偶发丢行；
                                       20ms（50Hz）留够余量。
                                       想更快看波形：把波特率提到 460800/921600 再把这里改小
                                       （整定速度环看 20~30Hz 振荡时必须改成 10 = 100Hz，
                                        否则 50Hz 采样会把它混叠成"慢摆"拍频） */

/* ===================== 调试开关（只改这里，不用动代码） =====================
 *   USER_PRINT_CSV  1 = 周期打印 16 通道 CSV（FireWater 上位机）；0 = 关闭
 *   TEST_MODE       上电校准链跑完后自动进入的测试：
 *                     0 = 不测（停在 STOP）      1 = 全开环（VVVF）
 *                     2 = 半开环（角度闭环+电压开环）  3 = 电流环（Id/Iq 闭环）
 *                     4 = 速度环（速度闭环+电流闭环，级联）
 *   TEST_IQ_A       电流环测试的 Iq 给定（A）：首次先用小值（0.1~0.2）试，
 *                   确认符号/方向没问题再提到额定 0.5A
 *   TEST_SPD_RPM    速度环测试的转速给定（rpm）
 *   TEST_IQ_MAX_USER 速度环测试的**用户限幅**（A，转矩限幅）：指定量，速度环输出的
 *                   Iq 给定不会超过它。实际生效限幅还要和**保底限幅**
 *                   （control.h 的 CURRENT_I_MAX_GUARD，0.9A，编译期常量、改不了）
 *                   取小 → 生效限幅 = min(用户限幅, 保底限幅)。两者分工：
 *                     用户限幅：你在这次运行里想给多大转矩（可随时改，默认 0.5A 额定）
 *                     保底限幅：硬件保护上限，谁都顶不破（必须 < 过流保护 1.2A）
 *   这两个量（转速 + 用户限幅）就是速度环+电流环模式下的"两个指定量"
 *   —— 串口接收命令后期开发（见 user_debug.h）：届时可在运行中直接改
 *      mc.target.spd_target / mc.target.iq_max，不必重新烧写
 *   以上参数的取值都写在 User_Test_Setup() 的对应分支里，改参数不用改结构 */
#define USER_PRINT_CSV     0        /* 电流环/速度环测试需要看 Id/Iq/Ud/Uq/iq_ref → 打开 */
#define TEST_MODE          4        /* 3 = 电流环；4 = 速度环（本次验证用） */
#define TEST_IQ_A          0.5f     /* 电流环测试给定（A）；额定 0.5A */
#define TEST_SPD_RPM       1000.0f  /* 速度环测试给定（rpm）：首次 1000，确认无误后再提 2000/2500 */
#define TEST_IQ_MAX_USER   0.5f     /* 速度环测试的用户限幅（A）：0.1~0.5 试；要更大就同时改
                                       control.h 的 CURRENT_I_MAX_GUARD（保底限幅，硬顶 0.9） */

static Led_Handle_t         g_led_G;          /* 绿色 LED 设备句柄 */
static volatile uint8_t  g_print_flag = 0;   /* 主循环打印标志 */
static uint16_t          g_print_cnt  = 0;   /* 打印周期计数 */
static volatile uint8_t  g_led_tick   = 0;   /* LED 1ms 节拍标志（中断置位、主循环消费） */
static volatile uint8_t  g_cfg_report = 0;   /* 测试配置一次性打印标志（确认烧进去的是哪套参数） */

/* ===================== 芯片中断回调函数 ===================== */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        /* 观测：进中断拉高 / 出中断拉低（TEST3 = PB3）
           脉冲宽度 = 注入中断处理耗时，示波器可据此测量 */
        TEST3(1);

        /* 20kHz 控制调度：一个分频调度器驱动高/中/低三环（注入 mc 实例） */
        Control_Sched(&mc);

        TEST3(0);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        /* LED 状态机不在这里跑了：它一次 70+ 条指令，还含 4 个运行期整数除法
           （M4 没有硬件除法，编译成库调用），而 LED 视觉上 1ms 的抖动无关紧要。
           中断里只置标志，实际更新挪到主循环（见 User_Loop） */
        g_led_tick = 1u;

        if (++g_print_cnt >= PRINT_PERIOD_MS)   /* 每 PRINT_PERIOD_MS(20ms) 置打印标志 */
        {
            g_print_cnt  = 0;
            g_print_flag = 1;
        }
    }
}

/* 串口接收：当前只发送；接收命令后期开发（届时用 HAL_UART_Receive_IT 单字节
   中断 + 主循环解析，可直接改 mc.target.spd_target / iq_max / 各环增益） */

/* ===================== 初始化与循环 ===================== */
void User_Init(void)
{
    /* ================= 系统级性能/抖动设置（放在这里，避免被 CubeMX 重新生成覆盖） =================
     * 1) 中断优先级：控制环（ADC1 注入转换回调 = 20kHz FOC）必须最高；
     *    通信类（SPI3-RX DMA = 编码器、USART1 + 其 TX DMA = 打印）降一级。
     *    原来它们和 ADC 同为优先级 0：同优先级不能互相抢占，于是编码器 DMA 回调里
     *    那 18 次逐位 CRC 循环（~0.75us）会原样变成控制环的抖动。
     *    降级后控制环抖动≈0，代价只是编码器帧结束晚几微秒（无害）。
     * 2) Flash 预取：170MHz + 4 等待周期下 ST 建议开启；本工程 ICACHE/DCACHE 是复位默认
     *    开着的，只有预取在 stm32g4xx_hal_conf.h 里被设成 PREFETCH_ENABLE=0。
     *    HAL 在 Flash 擦写时会自行关闭再恢复，所以在用户代码里补一句是安全的。 */
    HAL_NVIC_SetPriority(ADC1_2_IRQn,        0, 0);   /* 控制环：最高 */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);   /* 编码器 SPI3-RX DMA */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);   /* 打印 USART1-TX DMA */
    HAL_NVIC_SetPriority(USART1_IRQn,        1, 0);   /* 打印 USART1 */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

    /* ----- LED0 绑定 BSP 通道 0（BSP 绑定在设备层内部完成） ----- */
    Led_Init(&g_led_G, 0);    

    /* ----- 调试串口（发送：DMA） ----- */
    Debug_Usart_Init();

    /* ----- 启动 1ms 定时中断 ----- */
    HAL_TIM_Base_Start_IT(&htim6);

    /* ----- LED0 开始呼吸 ----- */
    Led_SetBreathe(&g_led_G, 255, 50, 2500, LED_WAVE_SINE);

    /* ----- 控制层：设备初始化（PWM/位置/电流，句柄写入 mc.device） -----
       模式为 Database 共享数据 mc.mode（初始停止），直接切换 */
    Control_Init(&mc);
}

/* ===================== 测试入口（调试用，开发完成后移除） =====================
   选哪个测试看文件顶部 TEST_MODE；每个分支 = 一组参数 + 一个模式。
   判断规则：
   1. 上电校准链（电流校准 → 编码器校准 → 停止）执行期间不干预模式；
   2. 仅当系统就绪（当前停止 + 电流校准完成 + 编码器已校准）才允许切入测试模式 */
static void User_Test_Setup(void)
{
    static uint8_t cfg_once = 0u;   /* 配置只上报一次 */

    if (mc.mode != MODE_STOP || !mc.device.cur.calib_done || !mc.enc_calib_done) {
        return;     /* 未就绪：不动模式（校准链自己在跑） */
    }
    if (mc.enc_calib.suspect != 0u) {
        return;     /* 校准结果**可疑**（原因见 CAL 行）：不自动进测试 ——
                       宁可停在 STOP 让人看 CAL 行，也不要拿一个坏角度去跑速度环 */
    }
    if (mc.oc_latched != 0u) {
        return;     /* 已触发过流保护：不再自动切入测试（重新上电才清） */
    }

    if (cfg_once == 0u) {           /* 首次就绪：打印一次实际生效的配置 */
        cfg_once     = 1u;
        g_cfg_report = 1u;
    }

#if (TEST_MODE == 1)
    /* ---- 全开环（电压/频率开环驱动）+ 编码器校准验证观测 ----
       用**实测参数**（R=3.5Ω、ψf=0.0034Wb，母线 12V）折算的量级参考：
         0.05 调制比 = 0.3V 相电压 → 堵转电流 ≈0.086A；空载转速 ≈120rpm（很慢）
                                      （扣掉摩擦电流 ~0.03A 的 R·i 压降后 ≈78rpm）
         0.30 调制比 = 1.8V        → 堵转电流 ≈0.51A（= 额定 0.5A）
       注：早期注解"0.3 曾致 3.x A"不对（当时电流 bias 没采对，读数≈常数 3.3A）；
           早期这里写的 R=4.2Ω/ψf=0.0037Wb 是后来的实测值推翻前的旧数（见 control.h） */
    mc.target.ol_volt = 0.05f;      /* Uq 电压幅值（调制比） */
    mc.target.ol_freq = 5.0f;       /* 电频率 (Hz) */
    mc.mode = MODE_FULL_OPEN_LOOP;

#elif (TEST_MODE == 2)
    /* ---- 半开环（角度闭环 + 电流开环） ----
       角度已带方向（两点锁轴校准自动定 dir），映射正确时能自持旋转。
       ⚠ 判据是**波动**而不是 Id 的均值：电压在本坐标系里直接给定、电流也投影回本坐标系，
         所以只要电机跟着走，Id 的均值天然 ≈0 —— **Id≈0 并不能证明角度对**（同一个道理见
         TEST_MODE 4 的校验基线）。真正的判据：
           平稳旋转、ch7/ch8 平稳（≈直流）⇒ 对齐正确；
           Id、Iq **明显波动/电流卡顿** ⇒ offset 有误（观察值随之起伏）；
           不转只抖、平均转矩 0 ⇒ dir 错（或校准未完成、CAL 行 SUSPECT）。
       此时若电机不转只是"电压不够/机械卡"，可小步上调 uq_target。
       注意：转子是自由的时，0.04 调制比(0.24V) 就会把它带到 ~100rpm 空载转速，
       电流随之掉到摩擦值（~0.03A 量级：1000rpm 下闭环实测 0.033A，见 TEST_MODE 4）
       ——看着像"没电流/堵转"，其实在转。
       单位：ud/uq_target 是调制比（1 = Udc/2），不是伏特 */
    mc.target.ud_target = 0.0f;     /* Ud 给定（调制比，通常 0） */
    mc.target.uq_target = 0.04f;    /* Uq 给定（调制比） */
    mc.mode = MODE_HALF_OPEN_LOOP;

#elif (TEST_MODE == 3)
    /* ---- 电流环（角度闭环 + 电流闭环）----
       测试要点：
       1) 先确认上电 CAL 行是 `dir=… ok`（角度/方向必须已校准，电流环用它做 Park）；
       2) 必须把轴**夹死**再看稳态——转子自由时电流环会把电机加速到电压顶速
          （空载顶速 = (u_max×Udc/2 − R·i_q)/ψf ⇒ 用现参数 = (1.10×6 − 3.5×0.06)/0.0034
            ≈ 2564rpm，✓实测 2578rpm，见 docs 笔记 2.7/E7），
          此时电流只剩摩擦值（0.03~0.06A 量级），看着"跟踪不上"其实是环已经饱和；
       3) 看 ch7/ch8（Id/Iq，A）与 ch9/ch10（Ud/Uq，调制比）：
          期望 Iq ≈ +TEST_IQ_A、Id ≈ 0（⚠ Id 在本坐标系下**天然 ≈0** ⇒ 它只确认采样零点与
          环路收敛，**不确认零位角**；堵转时角度误差看不出来，要判断角度得让电机转起来看 ch9）；
          堵转稳态 Uq ≈ R·Iq/(Udc/2)
          ✓已实测（母线 12V）：R=3.5Ω、0.5A → Uq=0.292（实测 Iq=0.508 / Uq=0.292、Id=0.000）；
       4) 若立刻出现 `FAULT: OC trip` → 电流反馈符号反了：
          把 drv8311.h 的 DRV8311_I_SIGN 改成 -1.0f，重新上电再试；
       5) 给定会被电流环的**保底限幅** CURRENT_I_MAX_GUARD（0.9A）逐轴钳位——要测更大
          电流就改 control.h 的 CURRENT_I_MAX_GUARD，但必须明显低于 CURRENT_I_TRIP(1.2A) */
    mc.target.id_target = 0.0f;         /* Id 目标（A，通常 0） */
    mc.target.iq_target = TEST_IQ_A;    /* Iq 目标（A，额定 0.5A；超保底限幅会被钳） */
    mc.mode = MODE_CURRENT_LOOP;

#elif (TEST_MODE == 4)
    /* ---- 速度环（速度闭环 + 电流闭环，级联）----
       本模式有**两个指定量**（都在 mc.target 里，改完重新烧写前一直有效；
       以后接了串口命令就能运行中直接改）：
         spd_target —— 转速目标（rpm）
         iq_max     —— **用户限幅**（A，转矩限幅）；实际生效限幅还要与保底限幅
                       CURRENT_I_MAX_GUARD（0.9A，编译期常量）取小
       速度环每拍用这两个量算 Iq 给定，输出的 Iq 不会超过 min(用户限幅, 保底限幅)。

       测试要点：
       1) 前提同电流环：上电 CAL 行必须 `dir=… ok`（角度错 → 速度环跑飞，靠 OC 保护兜底）；
       2) 转子**自由**转（不要夹死！夹死时速度环顶到 iq_max 堵转，就退化成电流环测试了）；
       3) 看 ch2(rpm)/ch14(spd_ref)/ch15(iq_ref)/ch8(Iq) 四条：
          期望 rpm 平滑跟到 spd_target；加速段 iq_ref 贴到 iq_max 或按斜坡走，
          稳态 iq_ref ≈ 摩擦电流（几十 mA 量级）、rpm 静差 ≈0；
       4) 整定顺序（改 control.h 的 SPEED_BW_HZ / MOTOR_INERTIA 重算，或改完初值后
          在运行中直接写 mc.speed.pid.kp / ki / ramp_rpm_s —— 串口命令后期开发，
          目前改参数要重新烧写）：
          a. 先看给定斜坡：默认 3000rpm/s 对应加速力矩 J·α = 3e-6×314.16 ≈ 9.4e-4N·m
             ≈ 0.026A（+摩擦 ~0.035A → **斜坡段 iq_ref 预期 ≈0.061A**），远低于 0.5A 限幅，
             PI 全程在线性区 → 首次验证看这条最干净（注意 3000rpm/s 只有能力的 5.3%，
             想激励环路动态要把 ramp 设 0 阶跃或 20000，那时 iq_ref 会顶到限幅）；
          b. 加 kp：直到 rpm/iq_ref 出现持续振荡（约 20~30Hz 的嗡声），
             再退回该值的 0.5~0.7 倍；
          c. 加 ki：目标 ki/kp = ωsc·SPEED_KI_RATIO = 2π×10×0.25 ≈ 15.7（BW 取 30 时 ≈47）
             （ki 太小 → rpm 有静差、恢复慢；ki 太大 → 超调/低频摆动）；
          d. 若 rpm 往反方向跑（越给越远）→ 不是速度环的问题：先回 TEST_MODE 3 夹死，
             确认 Iq 符号与角度方向（见 ch7/ch8 与 CAL 行 dir）；
       5) 带宽上限由反馈滤波决定：mech_speed_filt 是 fc≈95Hz 一阶低通 →
          SPEED_BW_HZ 取 20~40 才有相位裕度（不是 kp 能突破的）；
       6) 若一直 `FAULT: OC trip`：先确认用户限幅明显低于 CURRENT_I_TRIP(1.2A)，
          再看电流反馈符号（DRV8311_I_SIGN）；
       7) 两道限幅都不够用时（例如想短时给 0.8A）：改 control.h 的 CURRENT_I_MAX_GUARD
          （保底限幅）并把 TEST_IQ_MAX_USER 提上去，注意保底限幅必须 < 1.2A

       8) ---- 校验基线（J 已改为实测 3e-6、BW=10Hz 后，烧进去该看到什么）----
          · CFG1 行应显示 J=3.0e-6；CFG2 行的 spd 段应显示 kp≈0.00055 ki≈0.00869
            —— 先把这两行对一遍，确认"算出来的增益"就是"想验的增益"（算错的话后面全白看）；
          · ch15 斜坡段（200~800rpm，约 4~10 个点）预期 ≈0.061A
            = J·α/Kt(0.026A) + 摩擦(~0.035A)：**只由物理决定、与 kp/ki 无关**
            ⇒ ✓**已实测 0.061A**（与预期逐位吻合）⇒ J 与 Kt 这一组参数自洽；
          · ch15 稳态段预期 ≈摩擦电流：**✓ 已实测 0.033A**（1000rpm）
            ⇒ 反推 J = (0.061 − 0.033)·Kt/314 = 3.18e-6（与填入的 3.0e-6 差 6%，
              差值来自"斜坡段平均转速 ~500rpm 的摩擦 ≠ 1000rpm 的摩擦"，在电流分辨率
              1.6mA 的量级内 ⇒ 不必再改 MOTOR_INERTIA）；
          · ch2 应平滑跟到 1000rpm 且**无持续振荡**——896~1100 那种恒幅摆动必须消失
            ✓已实测：1000±20rpm（±2%）。判别这 ±20 是"环路振荡"还是"测量噪声底"：
              编码器 14bit ⇒ 1LSB=0.022°，转速按 100µs 差分 ⇒ 原始量化台阶 36.6rpm，
              经 95Hz 低通后本就落在 ±10rpm 量级（再被 50Hz 采样混叠）⇒ 更像噪声底。
              三个判别法：① 看 ch15 的 min/max——ch2 在摆而 ch15 基本平 ⇒ 噪声
              （10Hz 带宽的环跟不了 95Hz 以上的噪声）；② 改动 kp 2~3 倍，摆幅明显变
              才是环路；③ 改 TEST_SPD_RPM，量化噪声换算到 rpm **与转速无关**（恒 36.6rpm
              原始台阶）⇒ 500/2000rpm 下仍是 ±20 ⇒ 噪声；
          · ch2 稳态静差 ≈0（有积分器）；ch7(Id) ≈0 是**必然的**（环路用的是同一套角度，
            只证明电流环在自己坐标系里收敛，证明不了角度对——角度误差看 ch9）；
          · ch10(Uq) 稳态：**转速下由反电动势主导，不是堵转公式**——
            u_q(调制比) = [R·i_q + ωe·ψf]/(Udc/2)，ωe = pp·2π·n/60
            1000rpm 时 = [3.5×0.033 + 733×0.0034]/6 = [0.12 + 2.49]/6 ≈ **0.435**
            （✓已实测 0.431，差 0.9% ← Udc 用名义 12V 的误差；这一条同时反算 ψf：
              ψf = (u_q·Udc/2 − R·i_q)/ωe ≈ 0.00337Wb，与 0.0034 一致 ⇒ 低转速下
              独立复核了 ψf，此前只有顶速两个点）
            注意堵转时 (ωe=0) 才是 u_q = R·i_q/(Udc/2)（E5 测 R 用的就是那个）；
          · ch9(Ud) 稳态才是**角度探针**——ch7/ch8 不是：环路用的是同一套角度，
            Id≈0 只说明电流环在自己坐标系里干得对，**证明不了角度对**（角度误差只会
            把反电动势漏到 d 轴 ⇒ 落在 ch9 上）：
            δ = atan2(u_d' + ωe·Lq·i_q , u_q' − R·i_q)，u_d' = u_d·(Udc/2)
            1000rpm 实测 ch9 = −0.027 ⇒ δ ≈ **−2.97°**（其中交叉耦合项只有 −0.006）
            ⇒ 落在"上电零位残余 3~5°（齿槽，每次随机）"的量级内，不是延迟补偿错；
              但**单点分不开**"零位残余"与"延迟误差"（笔记 9.3）：要拆开必须在
              **同一次上电内**扫转速，再对 δ(ωe) 线性回归取截距/斜率。
            影响很小（cos3°=0.9986：转矩损 0.13%、多占 d 轴电压 0.128V），
              但顶速时这 3° 要吃电压余量 ⇒ 想再拿回几十 rpm 得做慢速旋转标定；
          · 量化动态（上升时间/超调）必须提高采样率：把 PRINT_PERIOD_MS 临时改 10
            （50Hz 下 35ms 的上升时间只有 1~2 个点，量不出来） */
    mc.target.spd_target = TEST_SPD_RPM;        /* 指定量 1：转速目标（rpm） */
    mc.target.iq_max     = TEST_IQ_MAX_USER;    /* 指定量 2：用户限幅（A，转矩限幅） */
    mc.target.id_target  = 0.0f;                /* 不参与：速度模式下 d 轴给定由速度环分支写 0 */
    /* 注意 target.iq_target 这里不用给值：速度模式下电流给定由
       Mode_SpeedCurrentLoop 每拍写进 mc.state.iq_ref（外环输出→内环给定） */
    mc.mode = MODE_SPEED_CURRENT_LOOP;
#endif
}

void User_Loop(void)
{
    /* LED 状态机（1ms 节拍，由 TIM6 中断置标志）：放在主循环，
       不在中断里做除法和那 70+ 条指令 —— LED 的毫秒级抖动肉眼看不出来 */
    if (g_led_tick)
    {
        g_led_tick = 0u;
        Led_Update(&g_led_G, TICK_FREQ_HZ);
    }

    /* 测试入口：模式与参数设置（带就绪判断，不打断上电校准链） */
    User_Test_Setup();

    /* 编码器校准结果一次性上报（人类可读单行，与周期 CSV 区分）。
       两种校准方式（control.h 的 ENC_CALIB_METHOD）打印的字段不同：
         方式一 两点锁轴：dmech = 两点机械位移、expect = 理论位移 π/2/pp ⇒ 判方向是否可信；
         方式二 慢速旋转：resid = 齿槽/噪声残差峰峰值、
                         load  = 跟随负载角 δe（电角度）、
                         rF/rR = 正/反向**跟随性比值**（同步时 ≈1.000，顺带复核极对数）、
                         att   = 第几次尝试（>0 说明判据没过、自动重来过）
       ⚠ 看 rF/rR 是判断"这次校准到底可不可信"的关键：失步按极对数量子化，
         丢一步 = 比值 0.857（窗口 ±2% 会抓住它）⇒ offset 偏差可达 180° 电角度，
         必须重烧/复位重测，不能拿去跑电机。
       dir=-1 表示编码器计数方向与电机相序相反，已在角度映射里自动纠正——
       这正是"锁轴对齐做了、半开环却不转"的原因；
       SUSPECT 的原因由 EncCalib_FailStr 给出（align 没静止 / 没转够 / 丢步 / 负载角），
       对应排查：校准电压够不够、机械是否卡、编码器是否在更新。
       串口忙则本轮不清标志、下一轮重试，避免与周期打印撞车丢消息 */
    if (mc.calib_report)
    {
#if (ENC_CALIB_METHOD == ENC_CALIB_METHOD_ROT)
        if (uart_printf("CAL: off=%.4f dir=%+d [ROT] resid=%.2fdeg load=%.2fdeg "
                        "rF=%.3f rR=%.3f att=%u first=%s emax=%u %s\n",
                        mc.device.pos.elec_offset,
                        (int)mc.device.pos.elec_dir,
                        mc.enc_calib.rot_resid * 57.29578f,                             /* rad→deg */
                        mc.enc_calib.rot_load * 57.29578f,                             /* rad→deg */
                        mc.enc_calib.rot_ratio_fwd,
                        mc.enc_calib.rot_ratio_rev,
                        (unsigned)mc.enc_calib.rot_attempt,
                        EncCalib_FailCode(mc.enc_calib.rot_fail_first),  /* 首次为何重试 */
                        (unsigned)mc.device.pos.miss_max,   /* emax：编码器最长断帧（≥200 ⇒ 冻结过） */
                        EncCalib_FailStr(&mc.enc_calib)))
        {
            mc.calib_report = 0u;
        }
#else
        if (uart_printf("CAL: off=%.4f dir=%+d dmech=%.4f expect=%.4f %s\n",
                        mc.device.pos.elec_offset,
                        (int)mc.device.pos.elec_dir,
                        mc.enc_calib.dmech,
                        (3.141592653589793f * 0.5f) / (float)MOTOR_POLE_PAIRS,
                        mc.enc_calib.suspect ? "SUSPECT(rotor-not-moved)"
                                             : "ok"))
        {
            mc.calib_report = 0u;
        }
#endif
    }

    /* 本次测试配置一次性上报：确认烧进去的到底是哪套参数（改宏后忘了重新下载时一眼可见）。
       分两行打印：TX_BUF_LEN=192，一整行塞不下（uart_printf 超长会截断 → 破坏整行） */
    if (g_cfg_report == 1u)
    {
        if (uart_printf("CFG1: TEST_MODE=%d spd*=%.0frpm iq*=%.2fA imax*=%.2fA id*=%.2fA | "
                        "R=%.2fohm L=%.2fmH psi=%.4fWb J=%.1fe-6 Udc_nom=%.1fV\n",
                        (int)TEST_MODE, mc.target.spd_target, mc.target.iq_target,
                        mc.target.iq_max, mc.target.id_target,
                        MOTOR_R_PHASE, MOTOR_LD * 1000.0f, MOTOR_FLUX,
                        MOTOR_INERTIA * 1.0e6f, MOTOR_UDC_NOMINAL))
        {
            g_cfg_report = 2u;      /* 第一行发完，下一轮发第二行 */
        }
    }
    else if (g_cfg_report == 2u)
    {
        if (uart_printf("CFG2: cur: umax=%.2f guard=%.2fA BW=%.0fHz FF=%d kp=%.3f ki=%.0f | "
                        "spd: kp=%.5f ki=%.5f imax*=%.2fA(us) eff=%.2fA ramp=%.0frpm/s BW=%.0fHz\n",
                        mc.current.u_max, CURRENT_I_MAX_GUARD, CURRENT_BW_HZ, (int)mc.current.ff_en,
                        mc.current.kp, mc.current.ki,
                        mc.speed.pid.kp, mc.speed.pid.ki, mc.target.iq_max,
                        mc.speed.iq_max, mc.speed.ramp_rpm_s, SPEED_BW_HZ))
        {
            g_cfg_report = 0u;
        }
    }

    /* 故障一次性上报（如电流环过流保护触发）：与周期 CSV 区分，串口忙则下轮重试 */
    if (mc.fault_report)
    {
        if (uart_printf("FAULT: OC trip, |I|>%.1fA -> STOP (power-cycle to re-arm)\n",
                        CURRENT_I_TRIP))
        {
            mc.fault_report = 0u;
        }
    }

    /* 驱动器故障引脚（nFAULT）：故障时芯片直接关断输出级——这是"给了电压却不走电流"
       的常见原因，原固件完全看不到它。状态变化时打印一次（故障消失后可再报） */
    {
        static uint8_t fault_latched = 0u;
        uint8_t nfault = MotorPwm_FaultRead(&mc.device.pwm);

        if (nfault != 0u) {
            if (fault_latched == 0u &&
                uart_printf("FAULT: DRV8311 nFAULT asserted (PC6 low) - output stage off\n")) {
                fault_latched = 1u;
            }
        } else {
            /* 故障**消失**也报一次，并带上"持续了多久、一共几次"：
                 短促一闪（几 ms）+ 次数多 ⇒ 电源跌落/接插件接触/干扰
                 较长（几十~几百 ms）+ 次数少 ⇒ 过流/过温自动重试
               这两类排查方向不同，所以这两个数字比"发生过故障"本身更有用 */
            if (fault_latched != 0u &&
                uart_printf("FAULT: DRV8311 nFAULT cleared (recovered) - was low %ums, %u times "
                            "-> check VM sag / OCP / OTSD\n",
                            (unsigned)(mc.drv_fault_ticks / 20u),   /* 20kHz ⇒ 20 拍 = 1ms */
                            (unsigned)mc.drv_fault_cnt)) {
                fault_latched = 0u;
            }
        }
    }

    /* 编码器数据冻结一次性上报：这是"电机不转、但一直有功率"的头号嫌疑
       （角度不更新 ⇒ 磁场不转 ⇒ 有电流没转矩）。判据：连续 >20ms 收不到新的有效帧。
       出现它就要查链路本身：SSI 接线/时钟、CRC 连续失败、DMA 卡死、编码器供电。
       miss_max = 本次上电以来"连续无新帧"的最大值（直接反映断了多久，10000 ≈ 1s）；
       ok = 累计收到的有效帧数（正常在 10kHz 下一路增长，冻结时停住） */
    {
        static uint8_t enc_stale_latched = 0u;

        if (PosSensor_IsStale(&mc.device.pos) != 0u) {
            if (enc_stale_latched == 0u &&
                uart_printf("FAULT: encoder stale (no valid frame >20ms, miss_max=%u, ok=%u) "
                            "- check MISO(PC11)/SCK(PC10)/CSN(PA15) & magnet\n",
                            (unsigned)mc.device.pos.miss_max,
                            (unsigned)mc.device.pos.frame_ok)) {
                enc_stale_latched = 1u;
            }
        } else {
            enc_stale_latched = 0u;
        }
    }

    /* 编码器磁场诊断（边上报，不污染周期 CSV 流）：MT6701 帧里自带"磁场状态 + 失磁标志"，
       以前驱动只取角度、把这两项丢了。**这是判断"角度读数冻结"根因的最直接证据**：
         · status≠0（过强/过弱）或 loss=1 ⇒ 磁铁丢了/离太远/太近 ⇒ 角度输出会冻结（CRC 仍正确）
         · 两项都正常而角度仍不跟着轴动 ⇒ 没有重新锁存 ⇒ 查 CSN/SPI 时序与接线
       变化时（含上电首次）打印一行；手转电机轴时若这行出现变化，就是磁铁/机械的问题 */
    {
        static uint8_t mag_last = 0xFFu;
        uint8_t mag_now = (uint8_t)((mc.device.pos.mag_loss << 2) | (mc.device.pos.mag_status & 0x3u));

        /* 只有**真正收到过有效帧**时这两位才有意义（恒 0 的帧已被 BSP 显式拒绝，
           否则它的 status/loss 也是 0，会被误读成"磁场正常"）。
           判读：status/loss 有异常 ⇒ 磁铁（丢失/太远/太近）；
                 两者都正常而角度不跟着轴动 ⇒ 通信/锁存（查 MISO-PC11/SCK-PC10/CSN-PA15） */
        if ((mag_now != mag_last) && (mc.device.pos.frame_ok != 0u)) {
            if (uart_printf("MAG: status=%u loss=%u frames=%u (0=磁场正常; !=0 ⇒ 磁铁异常)\n",
                            (unsigned)mc.device.pos.mag_status,
                            (unsigned)mc.device.pos.mag_loss,
                            (unsigned)mc.device.pos.frame_ok)) {
                mag_last = mag_now;
            }
        }
    }

    /* "编码器活着、但读数冻结"专用提示（一次性）：
       帧一直在收（不 stale、也不是全 0，BSP 已拒绝全 0）却始终没有位置变化 ⇒
       既不是驱动问题也不是失步，而是**位置反馈本身是死的**。这类故障最坑：
       所有其他判据看起来都"正常"，只有"转子转了而读数不动"才能发现它。
       两根线索：轴上磁铁是否随轴转 / CSN(PA15) 是否被拉死（芯片不重新锁存 ⇒ 老帧照发） */
    {
        static uint8_t frozen_latched = 0u;
        uint8_t frozen = (uint8_t)((mc.enc_calib.done != 0u) &&
                                   (mc.enc_calib.suspect != 0u) &&
                                   (mc.enc_calib.rot_ratio_fwd < 0.05f) &&
                                   (mc.enc_calib.rot_ratio_rev < 0.05f) &&
                                   (PosSensor_IsStale(&mc.device.pos) == 0u));

        if (frozen != 0u) {
            if (frozen_latched == 0u &&
                uart_printf("FAULT: encoder data flows but the angle NEVER changes (rF=rR≈0, not stale)\n"
                            "       -> the position feedback itself is dead: check the magnet on the\n"
                            "          shaft (must rotate with it) and CSN(PA15) (stuck low = old frame))\n")) {
                frozen_latched = 1u;
            }
        } else {
            frozen_latched = 0u;
        }
    }

    /* 周期打印：FireWater 纯数值逗号分隔 + 换行结尾
       ch0=Mech(rad) ch1=Elec(rad) ch2=Rpm ch3=Pos(rad)
       ch4=Ia(A) ch5=Ib(A) ch6=Ic(A)
       ch7=Id(A) ch8=Iq(A) ch9=Ud ch10=Uq（电流环反馈/输出）
       ch11=Du ch12=Dv ch13=Dw（最后写入 PWM 的三相占空比，读设备句柄缓存）
       ch14=spd_ref(rpm，速度环目标=target.spd_target) ch15=iq_ref(A，**送进电流环的 q 轴给定**
            = mc.state.iq_ref：电流环模式=target.iq_target，速度模式=速度环输出；
            速度模式下看它是否顶在**实际生效限幅** min(用户限幅, 保底限幅)
            （= mc.speed.iq_max，每拍刷新）可知是否限流）
       坐标系：ch0~ch3 全部为**电机坐标系**（已含 elec_dir），正方向 = 电角度增大方向，
       因此 ch0 与 ch1 一定同向（各绕 1 圈 / 7 圈）；编码器原始机械角（可能与 ch1 反向）
       要看得用 PosSensor_GetMechAngle()，上电 CAL 行会给出 dir 与两点位移
       （串口接收命令后期开发，当前只发送） */
    if (g_print_flag)
    {
        g_print_flag = 0;

#if (USER_PRINT_CSV)
        PosSensor_Handle_t     *pos = &mc.device.pos;   /* 设备句柄由 db 统一管理 */
        CurrentSensor_Handle_t *cur = &mc.device.cur;
        MotorPwm_Handle_t      *pwm = &mc.device.pwm;

        uart_printf("%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f\n",
                    PosSensor_GetMechAngleMotor(pos),
                    PosSensor_GetElecAngle(pos),
                    PosSensor_GetSpeedRpm(pos),
                    PosSensor_GetPosition(pos),
                    CurrentSensor_GetIa(cur),
                    CurrentSensor_GetIb(cur),
                    CurrentSensor_GetIc(cur),
                    mc.state.id,
                    mc.state.iq,
                    mc.state.ud,
                    mc.state.uq,
                    pwm->duty[0],
                    pwm->duty[1],
                    pwm->duty[2],
                    mc.target.spd_target,      /* ch14：速度环目标（rpm） */
                    mc.state.iq_ref);          /* ch15：送进电流环的 q 轴给定（A，已限幅） */
#endif
    }
}
