/**
 * @file    mc_current.c
 * @brief   电流环控制器实现（dq PI + 前馈解耦 + 矢量限幅 + 抗饱和）
 * @note    推导与单位见 mc_current.h
 */
#include "mc_current.h"

/* ==================== 内部工具 ==================== */

/** 限幅（无 libm） */
static float ClampF(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/**
 * @brief  电压矢量幅值近似：|(x,y)| ≈ 0.9604·max + 0.3978·min
 * @note   无 libm（工程不链接数学库）。这组系数相对误差 <4%。
 *         旧用的 max+0.5·min 在斜方向最大**高估 11.8%**：实测顶速矢量 (−0.241,0.979)
 *         被判成 1.100（以为顶到限幅 1.10），真实幅值只有 1.008 —— 等于白扔 9% 电压、
 *         顶速少约 200rpm。换成这组系数后同一个矢量算得 1.036（准确）
 */
static float VecMag(float x, float y)
{
    float ax = (x < 0.0f) ? -x : x;
    float ay = (y < 0.0f) ? -y : y;

    return (ax > ay) ? (0.9604f * ax + 0.3978f * ay)
                     : (0.9604f * ay + 0.3978f * ax);
}

/* ==================== 对外接口 ==================== */

void CurrentLoop_Init(CurrentLoop_t *loop, float kp, float ki, float u_max, float i_max, float v2m)
{
    loop->kp    = kp;
    loop->ki    = ki;
    loop->u_max = u_max;
    loop->i_max = i_max;
    loop->v2m   = v2m;

    loop->ff_en = 0u;       /* 前馈默认关：参数没量准之前不开 */
    loop->ld    = 0.0f;
    loop->lq    = 0.0f;
    loop->flux  = 0.0f;

    CurrentLoop_Reset(loop);
}

void CurrentLoop_SetFeedforward(CurrentLoop_t *loop, float ld, float lq, float flux, uint8_t en)
{
    loop->ld    = ld;
    loop->lq    = lq;
    loop->flux  = flux;
    loop->ff_en = (en != 0u) ? 1u : 0u;
}

void CurrentLoop_Reset(CurrentLoop_t *loop)
{
    loop->integral_d = 0.0f;
    loop->integral_q = 0.0f;
}

void CurrentLoop_Run(CurrentLoop_t *loop, float dt,
                     float id_ref, float iq_ref, float id_fb, float iq_fb, float we,
                     float *ud, float *uq)
{
    float ed = id_ref - id_fb;          /* d 轴误差 (A) */
    float eq = iq_ref - iq_fb;          /* q 轴误差 (A) */
    float ud_ff = 0.0f;                 /* 前馈（调制比） */
    float uq_ff = 0.0f;
    float ud_raw, uq_raw;               /* 限幅前的 PI + 前馈输出（调制比） */
    float mag, scale;
    uint8_t sat;

    /* ---- 0. 电流给定限幅（逐轴）----
       电流上限是**电机/驱动**的属性，所以放在内环统一把关：不管给定来自用户（电流环模式）、
       速度环还是以后的位置环，都进不了这个门——外环各自设限迟早会漏一个。
       为什么逐轴而不是矢量限幅：矢量限幅会改变电流矢量方向（=转矩角），对"给定"来说
       不可预期；iq 就是转矩分量，限 iq 即限转矩，id 另限（本工程恒 0，不弱磁/不 MTPA） */
    if (id_ref > loop->i_max)       id_ref = loop->i_max;
    else if (id_ref < -loop->i_max) id_ref = -loop->i_max;
    if (iq_ref > loop->i_max)       iq_ref = loop->i_max;
    else if (iq_ref < -loop->i_max) iq_ref = -loop->i_max;

    /* ---- 1. 前馈解耦（可选）：把交叉耦合与反电动势一次补掉 ----
       u_d_ff = −ωe·L_q·i_q ；u_q_ff = +ωe·(L_d·i_d + ψf)   [V] → ×v2m 换成调制比 */
    if (loop->ff_en != 0u)
    {
        ud_ff = -we * loop->lq * iq_fb * loop->v2m;
        uq_ff =  we * (loop->ld * id_fb + loop->flux) * loop->v2m;
    }

    /* ---- 2. PI：比例直接作用，积分项用上一拍累加值（下拍再累加本拍误差，
               这样本拍输出与饱和判断用的是同一个积分值，抗饱和逻辑干净） ---- */
    ud_raw = loop->kp * ed + loop->integral_d + ud_ff;
    uq_raw = loop->kp * eq + loop->integral_q + uq_ff;

    /* ---- 3. 矢量限幅：超限等比缩小，保持电压矢量方向（不改变转矩角） ---- */
    mag   = VecMag(ud_raw, uq_raw);
    scale = (mag > loop->u_max) ? (loop->u_max / mag) : 1.0f;
    *ud = ud_raw * scale;
    *uq = uq_raw * scale;

    /* ---- 4. 抗饱和（条件积分）----
       未限幅 → 正常积分；
       已限幅 → 只有该轴误差方向与限幅输出方向相反（在"往回拉"）时才积分，
                否则冻结，避免深饱和时积分堆满、退饱和时大幅超调；
       积分再叠 ±u_max 硬限幅兜底（反馈符号接反时不至于无限发散） */
    sat = (scale < 1.0f) ? 1u : 0u;

    if ((sat == 0u) || ((ed * ud_raw) < 0.0f))
    {
        loop->integral_d = ClampF(loop->integral_d + loop->ki * dt * ed,
                                  -loop->u_max, loop->u_max);
    }
    if ((sat == 0u) || ((eq * uq_raw) < 0.0f))
    {
        loop->integral_q = ClampF(loop->integral_q + loop->ki * dt * eq,
                                  -loop->u_max, loop->u_max);
    }
}
