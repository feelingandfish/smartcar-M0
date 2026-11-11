/**
  * encoder.c —— 编码器测速 (TIM2 编码器模式, 4倍频)
  */
#include "encoder.h"

extern TIM_HandleTypeDef htim2;   /* TIM2 编码器句柄 (main.c 中定义) */

static uint16_t s_last_cnt = 0;

void Encoder_Init(void)
{
    s_last_cnt = __HAL_TIM_GET_COUNTER(&htim2);
}

float Encoder_GetRPM(void)
{
    uint16_t cnt = __HAL_TIM_GET_COUNTER(&htim2);
    int16_t  delta = (int16_t)(cnt - s_last_cnt);   /* 16位溢出自动处理 */
    s_last_cnt = cnt;

    /* 采样窗口 10ms:
       rpm = delta(计数) / 1400(每圈计数) * 60(s->min) / 0.01(s)
           = delta * 6000 / 1400 */
    return (float)delta * 6000.0f / (float)ENC_COUNTS_PER_REV;
}
