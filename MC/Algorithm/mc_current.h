/**
 * @file    mc_current.h
 * @brief   电流环控制器：dq 轴 PI + 前馈解耦 + 电压矢量限幅 + 抗饱和（平台无关）
 *
 * 被控对象（PMSM 在 dq 轴上的电压方程，交叉耦合项已在下面说明）：
 *   u_d = R·i_d + L_d·di_d/dt − ωe·L_q·i_q
 *   u_q = R·i_q + L_q·di_q/dt + ωe·L_d·i_d + ωe·ψf
 *
 * 本控制器四件事：
 *   1) PI：e = i_ref − i_fb，u_pi = kp·e + ki·∫e dt
 *      kp = L·ωc/(Udc/2)、ki = R·ωc/(Udc/2)（PI 零点对消电气极点 R/L，ωc = 2π·带宽）；
 *      ki 已按 dt 归一（单位 调制比/(A·s)），改控制频率不用重调增益。
 *   2) 前馈解耦（ff_en=1）：u_d_ff = −ωe·L_q·i_q、u_q_ff = +ωe·(L_d·i_d + ψf)，
 *      把转速相关的耦合与反电动势一次性补掉，PI 只管误差 → 高转速下跟踪不塌。
 *   3) 矢量限幅：|(u_d,u_q)| > u_max 时等比缩小，**保持电压矢量方向**（不改变转矩角；
 *      各轴独立限幅会改变矢量角度）→ 天然不过调制。
 *   4) 抗饱和：限幅期间冻结积分（若误差方向在"往回拉"则仍允许积分），
 *      并对积分做 ±u_max 硬限幅兜底——电流反馈符号万一接反，不至于无限发散。
 *
 * 单位约定（与工程其余部分一致）：
 *   电流 A；电压 = 调制比（1.0 = Udc/2 相电压峰值，**非 V**）；ωe = 电角速度 rad/s；
 *   前馈用的 L/ψf 是物理单位（H / Wb），内部用 v2m = 2/Udc 折算成调制比。
 *
 * @note 无 libm：矢量幅值用 max+0.5·min 近似（误差 < 4%），只用于限幅足够
 */
#ifndef __MC_CURRENT_H__
#define __MC_CURRENT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  电流环控制器句柄
 */
typedef struct {
    /* ---- 增益（运行中可直接改，立即生效） ---- */
    float kp;          /**< 比例增益（调制比/A）：kp = L·ωc / (Udc/2) */
    float ki;          /**< 积分增益（调制比/(A·s)）：ki = R·ωc / (Udc/2)，内部乘 dt */
    float v2m;         /**< 伏特 → 调制比 换算系数 = 2/Udc（前馈项用） */

    /* ---- 输出限幅 ---- */
    float u_max;       /**< dq 电压矢量幅值上限（调制比）：线性调制区上限 1.1547，**但实际可用
                            上限更低**——由低边电流采样窗口决定（见 control.h CURRENT_U_MAX）：
                            本项目取 1.10（实测顶速 2578rpm），需配合 drv8311_cur.c 把 ADC 触发点
                            放在计数器波峰，否则调制比 >1.04 时相电流读数会被开关沿污染 */
    float i_max;       /**< dq 电流给定限幅（A）：id_ref/iq_ref 进环前**逐轴**限幅（额定电流） */

    /* ---- 前馈解耦（可选） ---- */
    uint8_t ff_en;     /**< 1 = 使能前馈解耦；参数不准时先关掉（默认关） */
    float ld;          /**< d 轴电感 (H) */
    float lq;          /**< q 轴电感 (H) */
    float flux;        /**< 永磁磁链 ψf (Wb) */

    /* ---- 内部状态（外部勿改） ---- */
    float integral_d;  /**< d 轴积分累加（调制比） */
    float integral_q;  /**< q 轴积分累加（调制比） */
} CurrentLoop_t;

/**
 * @brief  初始化电流环（积分清零；前馈默认关闭，用 SetFeedforward 开）
 * @param  loop   句柄
 * @param  kp     比例增益（调制比/A）
 * @param  ki     积分增益（调制比/(A·s)）
 * @param  u_max  电压矢量幅值上限（调制比）
 * @param  i_max  电流给定限幅（A）：id_ref/iq_ref 逐轴限幅，保护电机与驱动
 * @param  v2m    伏特→调制比系数 = 2/Udc
 */
void CurrentLoop_Init(CurrentLoop_t *loop, float kp, float ki, float u_max, float i_max, float v2m);

/**
 * @brief  设置前馈解耦参数（en=0 时只存参数不参与计算）
 * @param  loop  句柄
 * @param  ld    d 轴电感 (H)
 * @param  lq    q 轴电感 (H)
 * @param  flux  永磁磁链 (Wb)
 * @param  en    1 = 使能前馈
 */
void CurrentLoop_SetFeedforward(CurrentLoop_t *loop, float ld, float lq, float flux, uint8_t en);

/**
 * @brief  积分清零（从停止切入电流环时调用，避免带上一次运行的积分残留）
 */
void CurrentLoop_Reset(CurrentLoop_t *loop);

/**
 * @brief  执行一拍电流环：给定与反馈 → 输出 dq 电压（调制比）
 * @param  loop     句柄
 * @param  dt       本拍时间步长 (s)：1/控制频率
 * @param  id_ref   d 轴电流给定 (A)
 * @param  iq_ref   q 轴电流给定 (A)
 * @param  id_fb    d 轴电流反馈 (A)
 * @param  iq_fb    q 轴电流反馈 (A)
 * @param  we       电角速度 (rad/s，正 = 电角度增大方向，前馈用；ff_en=0 时忽略)
 * @param  ud       输出：d 轴电压（调制比）
 * @param  uq       输出：q 轴电压（调制比）
 * @note   给定先逐轴限幅到 ±i_max（电流上限由内环统一把关，外环不必各自设限）；
 *         输出已做矢量限幅，可直接送 MC_SVPWM_Calc / 逆 Park
 */
void CurrentLoop_Run(CurrentLoop_t *loop, float dt,
                     float id_ref, float iq_ref, float id_fb, float iq_fb, float we,
                     float *ud, float *uq);

#ifdef __cplusplus
}
#endif

#endif /* __MC_CURRENT_H__ */
