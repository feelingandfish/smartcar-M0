#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;       /* 积分累计项 */
    float prev_error;     /* 上一次误差(用于微分) */
    float out_max;        /* 输出限幅上限 */
    float out_min;        /* 输出限幅下限 */
    float integral_max;   /* 积分限幅(抗积分饱和) */
    float d_filtered;     /* 微分项一阶低通后的值(不完全微分) */
    float d_alpha;        /* 微分低通系数 0~1, 越小滤得越狠 */
    float out;            /* 最近一次输出 */
} PID_t;

/* 初始化: kp/ki/kd, 输出限幅[out_min, out_max] */
void  PID_Init(PID_t *p, float kp, float ki, float kd, float out_max, float out_min);

/* 在线修改参数(现场调参用) */
void  PID_SetParam(PID_t *p, float kp, float ki, float kd);

/* 位置式 PID 计算: 返回控制量, 已做输出限幅 */
float PID_Compute(PID_t *p, float target, float actual, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
