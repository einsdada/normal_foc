/**
 * @file    mc_speed.h
 * @brief   速度环控制器：PI + 给定斜坡 + 输出限幅 → Iq 给定（平台无关）
 *
 * 级联位置：速度环（外环，10kHz）→ 电流环（内环，20kHz）
 *   速度环输出 = Iq 给定（A），电流环把它当 q 轴给定执行；Id 给定恒为 0
 *   （表贴式 PMSM，无磁阻转矩，不做 MTPA）。
 *
 * 被控对象（电流环已闭环后，"Iq 给定 → 机械转速"近似为一阶惯性/积分环节）：
 *   J·dωm/dt = Kt·iq − TL − B·ωm        Kt = 1.5·pp·ψf  [N·m/A]
 *   （1.5 来自幅值不变 Clarke/Park：三相合成转矩 3/2·pp·ψf·iq）
 *
 * 控制器与增益设计（PI 零点压在带宽以下，把积分型对象整定成 −20dB/dec）：
 *   iq_ref = kp·(ω_ref − ω_fb) + ki·∫(ω_ref − ω_fb)dt     限幅 ±实际限幅
 *   实际限幅 = min(用户限幅, 保底限幅)：前者是用户指定的转矩限幅（mc.target.iq_max），
 *   后者是硬件保护（iq_guard，Init 设定、运行中不可改）——见 SpeedLoop_Run
 *   ωsc = 2π·带宽 (rad/s)；kp = J·ωsc/Kt；ki = kp·ωsc/4（零点在 ωsc/4，相位裕度 ≈76°）
 *   ⇒ kp 决定带宽（J 越大越"重"，同样带宽要更大 kp）；ki 决定零点的位置。
 *
 * 单位：对外用 rpm（与 mc.target.spd_target、PosSensor_GetSpeedRpm 一致）。
 *   误差用 rpm 时 kp_rpm = kp_rads·(2π/60)、ki_rpm = ki_rads·(2π/60)；
 *   带宽/时间常数不随单位变（rpm 与 rad/s 只差常数倍）。
 *
 * ⚠ 两个限制（决定实际可用带宽，详见 control.h 速度环参数节）：
 *   1) 反馈 mech_speed_filt 带 fc≈95Hz 一阶低通（约 1.7ms 延迟），
 *      带宽取 20~40Hz 才有足够相位裕度——这是本环带宽的真正瓶颈，不是算出来的 kp；
 *   2) J（转子+负载转动惯量）必须实测或反推，不能拍脑袋：本项目已由斜坡段 iq_ref
 *      反推 J ≈ 3e-6 并填入 MOTOR_INERTIA（见 control.h），增益因此可直接由公式算出；
 *      仍在现场整定的场合是**负载变了**（kp ∝ J）：P 加到轻微振荡 → 退回 0.5~0.7 倍
 *      → I 加到 ki/kp = ωsc·SPEED_KI_RATIO。
 *
 * 给定斜坡（ramp_rpm_s）：让 ω_ref 按斜率逼近目标，使加速力矩 J·dω/dt 落在电流限幅内。
 *   不加速率限幅时阶跃给定会让 PI 直接顶到 ±iq_max = 力矩受限启动（积分饱和、
 *   到位时超调）；加斜坡后 PI 全程工作在线性区。ramp_rpm_s = 0 表示关闭（直接跟随）。
 *
 * @note 无 libm、无除法依赖：Run 内只有加/减/乘/比较，积分按 dt 归一（同 mc_current）
 */
#ifndef __MC_SPEED_H__
#define __MC_SPEED_H__

#include <stdint.h>
#include "pid.h"        /* PID_Handle_t：本环的 PI 内核（dt 归一、条件积分抗饱和） */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  速度环控制器句柄
 */
typedef struct {
    PID_Handle_t pid;        /**< PI 内核：kp (A/rpm)、ki (A/(rpm·s))、输出限幅 ±iq_max */
    float        ramp_rpm_s; /**< 给定斜率限幅 (rpm/s)：0 = 关闭（给定直接跟随目标） */
    float        ref;        /**< 斜坡后的给定 (rpm)：内部状态，调试可读 */
    float        iq_guard;   /**< 保底限幅 (A)：硬件保护上限，运行中不可改（Init 设定） */
    float        iq_max;     /**< 本拍**实际生效**的限幅 (A) = min(用户限幅, 保底限幅)，
                                  由 SpeedLoop_Run 每拍刷新，供外部读取/打印 */
} SpeedLoop_t;

/**
 * @brief  初始化速度环 
 * @param  loop         句柄
 * @param  kp           比例增益 (A/rpm)
 * @param  ki           积分增益 (A/(rpm·s)，内部乘 dt)
 * @param  iq_user_max  用户限幅默认值 (A)：实际每拍由 SpeedLoop_Run 的入参刷新
 *                      （来自用户目标 mc.target.iq_max），这里只是上电默认值
 * @param  iq_guard     保底限幅 (A)：硬件保护上限，运行中不可改
 * @param  ramp_rpm_s   给定斜率限幅 (rpm/s)：0 = 关闭
 * @note   积分与斜坡给定清零
 */
void SpeedLoop_Init(SpeedLoop_t *loop, float kp, float ki,
                    float iq_user_max, float iq_guard, float ramp_rpm_s);

/**
 * @brief  复位：积分清零 + 斜坡给定归零
 * @note   从停止切入速度环时调用（否则会带上一次运行的积分残留与旧给定）；
 *         斜坡给定归零 = 每次启动都从 0rpm 平滑升到目标
 */
void SpeedLoop_Reset(SpeedLoop_t *loop);

/**
 * @brief  执行一拍速度环：目标/反馈（rpm）→ Iq 给定（A）
 * @param  loop        句柄
 * @param  dt          本拍时间步长 (s)：1/调用频率（10kHz → 1e-4）
 * @param  spd_ref     速度目标 (rpm)
 * @param  spd_fb      速度反馈 (rpm)：用 PosSensor_GetSpeedRpm()（已滤波、电机坐标系）
 * @param  iq_user_max 用户限幅 (A)：用户指定的最大电流（mc.target.iq_max）。
 *                     本函数取 min(它, 保底限幅) 作为实际限幅，同时作为输出限幅与
 *                     条件积分的饱和判据基准 —— 所以即使有人直接写字段，
 *                     保底限幅也顶不破，且外环饱和判据始终与实际一致
 * @retval Iq 给定 (A)，已限幅到 ±实际限幅；直接交给电流环当 q 轴给定
 */
float SpeedLoop_Run(SpeedLoop_t *loop, float dt, float spd_ref, float spd_fb, float iq_user_max);

#ifdef __cplusplus
}
#endif

#endif /* __MC_SPEED_H__ */
