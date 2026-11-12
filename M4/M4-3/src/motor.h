#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 初始化: 方向脚(AIN1=PB0, AIN2=PB1) + 启动PWM(TIM3_CH1/PA6) */
void Motor_Init(void);

/*
 * 设置电机输出
 * pwm: -1000 ~ +1000  (正=正转, 负=反转, 0=停止/滑行)
 * 满量程1000 对应 100% 占空比
 */
void Motor_SetPWM(int32_t pwm);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
