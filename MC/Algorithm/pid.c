/**
 * @file    pid.c
 * @brief   PI 控制器实现（位置式 + 积分限幅）
 */
#include "pid.h"

void PID_Init(PID_Handle_t *pid, float kp, float ki, float out_min, float out_max)
{
    pid->kp       = kp;
    pid->ki       = ki;
    pid->out_min  = out_min;
    pid->out_max  = out_max;
    pid->integral = 0.0f;
    pid->out      = 0.0f;
}

float PID_Calc(PID_Handle_t *pid, float err, float dt)
{
    float i_term, out;
    uint8_t sat;

    /* ---- 1. 用上一拍累加好的积分先算本拍输出与饱和判断 ----
       本拍误差的积分下一拍才生效：这样"输出"与"饱和判断"用的是同一个积分值，
       条件积分的判据才干净（与 mc_current 的处理顺序一致） */
    i_term = pid->ki * pid->integral;
    out    = pid->kp * err + i_term;

    sat = 0u;
    if (out > pid->out_max)
    {
        out = pid->out_max;
        sat = 1u;
    }
    else if (out < pid->out_min)
    {
        out = pid->out_min;
        sat = 1u;
    }

    /* ---- 2. 条件积分 ----
       未饱和 → 正常累加；
       已饱和 → 只有误差方向与饱和输出方向相反（在"往回拉"）时才累加，否则冻结，
                避免深饱和时积分堆满、退饱和时大幅超调；
       积分再叠输出限幅兜底（误差符号接反时不至于无限发散）；
       ki=0（纯比例）→ 积分不累加，同时避开下面除以 ki */
    if (pid->ki == 0.0f)
    {
        pid->integral = 0.0f;
    }
    else if ((sat == 0u) || ((err * out) < 0.0f))
    {
        pid->integral += dt * err;
        if (pid->ki * pid->integral > pid->out_max)
        {
            pid->integral = pid->out_max / pid->ki;
        }
        else if (pid->ki * pid->integral < pid->out_min)
        {
            pid->integral = pid->out_min / pid->ki;
        }
    }

    pid->out = out;
    return out;
}
