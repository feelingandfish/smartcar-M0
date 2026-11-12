/**
  * motor.c —— PWM + 方向控制 (TB6612)
  *   PWMA = PA6 (TIM3_CH1)
  *   AIN1 = PB0
  *   AIN2 = PB1
  */
#include "motor.h"

extern TIM_HandleTypeDef htim3;   /* TIM3 PWM 句柄 (main.c 中定义) */

#define MOTOR_PWM_FULL   1000      /* 逻辑满量程(占空比百分比 ×10) */
#define MOTOR_ARR        99        /* TIM3 重载值: duty 0~99 => 0~100% */

void Motor_Init(void)
{
    /* 方向脚默认为低 = 停止 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}

void Motor_SetPWM(int32_t pwm)
{
    if (pwm >  MOTOR_PWM_FULL) pwm =  MOTOR_PWM_FULL;
    if (pwm < -MOTOR_PWM_FULL) pwm = -MOTOR_PWM_FULL;

    uint32_t duty = (uint32_t)((pwm < 0 ? -pwm : pwm) * (MOTOR_ARR + 1) / MOTOR_PWM_FULL);

    if (pwm > 0) {          /* 正转: AIN1=1, AIN2=0 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    } else if (pwm < 0) {   /* 反转: AIN1=0, AIN2=1 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    } else {                /* 停止: AIN1=0, AIN2=0 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
}
