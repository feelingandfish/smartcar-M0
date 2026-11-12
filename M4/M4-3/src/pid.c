/**
  * pid.c —— 位置式 PID (含输出限幅 + 条件积分抗饱和)
  *   u = Kp*e + Ki*∫e dt + Kd*de/dt
  */
#include "pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd, float out_max, float out_min)
{
    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
    p->integral = 0.0f;
    p->prev_error = 0.0f;
    p->out_max = out_max;
    p->out_min = out_min;
    p->integral_max = out_max;   /* 积分限幅 = 输出上限 */
    p->d_filtered = 0.0f;
    p->d_alpha = 0.3f;           /* 一阶低通: alpha=dt/(Tf+dt), Tf≈23ms */
    p->out = 0.0f;
}

void PID_SetParam(PID_t *p, float kp, float ki, float kd)
{
    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
}

float PID_Compute(PID_t *p, float target, float actual, float dt)
{
    float e, P, D_raw, D, out_try, out;

    e = target - actual;

    /* 比例项 */
    P = p->Kp * e;

    /* 微分项: 原始微分 dt 很小会放大编码器量化噪声,
       这里做一阶低通(不完全微分)抑制噪声 */
    D_raw = p->Kd * (e - p->prev_error) / dt;
    p->prev_error = e;
    p->d_filtered += p->d_alpha * (D_raw - p->d_filtered);
    D = p->d_filtered;

    /* 预判输出是否饱和 */
    out_try = P + p->integral + D;

    /* 条件积分抗饱和: 输出未饱和才累加积分, 并对积分限幅 */
    if (out_try <= p->out_max && out_try >= p->out_min) {
        p->integral += p->Ki * e * dt;
        if (p->integral > p->integral_max) p->integral = p->integral_max;
        if (p->integral < p->out_min)      p->integral = p->out_min;
    }

    out = P + p->integral + D;

    /* 输出限幅 */
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;

    p->out = out;
    return out;
}
