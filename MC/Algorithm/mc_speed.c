/**
 * @file    mc_speed.c
 * @brief   速度环控制器实现（PI + 给定斜坡 + 输出限幅）
 * @note    推导、单位与整定方法见 mc_speed.h
 */
#include "mc_speed.h"

void SpeedLoop_Init(SpeedLoop_t *loop, float kp, float ki,
                    float iq_user_max, float iq_guard, float ramp_rpm_s)
{
    loop->iq_guard   = (iq_guard > 0.0f) ? iq_guard : 0.0f;
    loop->iq_max     = iq_user_max;
    loop->ramp_rpm_s = (ramp_rpm_s > 0.0f) ? ramp_rpm_s : 0.0f;

    /* PI 内核：输出限幅取 ±iq_user_max（每拍还会被 SpeedLoop_Run 按 min(用户,保底) 刷新） */
    PID_Init(&loop->pid, kp, ki, -iq_user_max, iq_user_max);

    loop->ref = 0.0f;
}

void SpeedLoop_Reset(SpeedLoop_t *loop)
{
    loop->pid.integral = 0.0f;
    loop->pid.out      = 0.0f;
    loop->ref          = 0.0f;      /* 从 0rpm 重新爬升（软启动） */
}

float SpeedLoop_Run(SpeedLoop_t *loop, float dt, float spd_ref, float spd_fb, float iq_user_max)
{
    float lim = iq_user_max;

    /* ---- 0. 本拍的实际限幅 = min(用户限幅, 保底限幅)：立刻生效 ----
       保底限幅是编译期常量（硬件保护），即使有人绕过 Control_SetCurrentLimit()
       直接写 target.iq_max 也顶不破；用这同一个值刷新 PI 输出限幅，
       保证条件积分的饱和判据与实际限幅永远一致（否则退饱和时会超调） */
    if (lim > loop->iq_guard)
    {
        lim = loop->iq_guard;
    }
    if (lim < 0.0f)
    {
        lim = 0.0f;
    }
    loop->iq_max      = lim;
    loop->pid.out_max = lim;
    loop->pid.out_min = -lim;

    /* ---- 1. 给定斜坡：|Δref| ≤ ramp·dt，避免阶跃给定把 PI 直接顶到限幅 ----
       （关闭时直接跟随；开启时按斜率逼近，加速力矩 J·dω/dt 落在实际限幅内） */
    if (loop->ramp_rpm_s > 0.0f)
    {
        float step = loop->ramp_rpm_s * dt;
        float d    = spd_ref - loop->ref;

        if (d > step)
        {
            loop->ref += step;
        }
        else if (d < -step)
        {
            loop->ref -= step;
        }
        else
        {
            loop->ref = spd_ref;    /* 已追上：贴住目标 */
        }
    }
    else
    {
        loop->ref = spd_ref;
    }

    /* ---- 2. PI：误差用 rpm，输出 Iq 给定 (A)，限幅与抗饱和在 PID_Calc 内 ---- */
    return PID_Calc(&loop->pid, loop->ref - spd_fb, dt);
}
