#ifndef ENCODER_H
#define ENCODER_H
#include "main.h"
#include <stdint.h>
#define ENCODER_COUNTS_PER_REV 1385U /* 实测输出轴每圈计数 */
typedef struct {
    uint16_t previous_count;
    uint32_t previous_tick;
    int16_t delta_count;
    float rpm;
} Encoder_HandleTypeDef;
void Encoder_Init(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *timer, uint32_t now_ms);
HAL_StatusTypeDef Encoder_Start(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *timer);
uint8_t Encoder_Update(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *timer,
                       uint32_t now_ms, uint32_t *elapsed_ms);
#endif
