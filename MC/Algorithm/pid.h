/**
 * @file    pid.h
 * @brief   PI 控制器（位置式：条件积分抗饱和 + 积分/输出限幅），平台无关
 *
 * 用途：**外环** —— 速度环（spd_target → Iq 给定）、位置环（pos_target → 速度给定）。
 *       电流环不走这里：见 Algorithm/mc_current.c（dq 矢量控制器：
 *       PI + 前馈解耦 + 电压矢量限幅 + 抗饱和）。
 * 输出约定：输出是下一级给定，量纲由使用者定义（如速度环输出 A），限幅由使用者设置。
 * 抗饱和：条件积分（饱和且误差仍把输出往饱和方向推 → 冻结积分）+ 积分限幅兜底，
 *         与电流环 mc_current 用同一套写法。
 *
 * @note 积分的时间基准：PID_Calc 显式接收 dt，积分按 integral += dt·err 累加，
 *       ki 的量纲是"每秒"（如 A/(rpm·s)）→ **与控制频率/分频无关**，改频率不用重调
 *       （与电流环 mc_current 的 ki 归一方式一致）。
 *       dt=1.0f 调用即为"按调用次数累加"的老行为，但不要再这样用了。
 */
#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  PI 控制器句柄
 */
typedef struct {
    float kp;           /**< 比例增益 */
    float ki;           /**< 积分增益（量纲 = 输出/误差/秒） */
    float out_min;      /**< 输出下限（双极性，如 -0.3） */
    float out_max;      /**< 输出上限（如 +0.3） */
    float integral;     /**< 积分累加 ∫err·dt（内部状态，无需外部读写） */
    float out;          /**< 本次输出（内部状态） */
} PID_Handle_t;

/**
 * @brief  初始化 PI 控制器
 * @param  pid      句柄
 * @param  kp       比例增益（量纲由使用者定义，如 A/rpm）
 * @param  ki       积分增益（量纲 = 输出/误差/秒，如 A/(rpm·s)；0 = 纯比例）
 * @param  out_min  输出下限
 * @param  out_max  输出上限
 */
void PID_Init(PID_Handle_t *pid, float kp, float ki, float out_min, float out_max);

/**
 * @brief  执行一拍 PI 计算
 * @param  pid  句柄
 * @param  err  误差（给定 - 反馈）
 * @param  dt   本拍时间步长 (s)：1/调用频率（积分按 dt 归一，见文件头 @note）
 * @retval 限幅后的输出
 * @note   每个控制周期调用一次；参数（kp/ki/限幅）可在运行中直接改句柄字段
 */
float PID_Calc(PID_Handle_t *pid, float err, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
