#ifndef FILTER_H
#define FILTER_H
#include "mpu6050.h"
#include <stdint.h>
#define FILTER_GYRO_CALIBRATION_SAMPLES 200U
#define FILTER_COMPLEMENTARY_ALPHA 0.99f
typedef struct {
    uint16_t calibration_count;
    int32_t gyro_sum_x_raw, gyro_sum_y_raw, gyro_sum_z_raw;
    uint8_t calibrated;
    uint8_t angle_initialized;
    float gyro_bias_x_dps, gyro_bias_y_dps, gyro_bias_z_dps;
    float acc_pitch_deg;
    float gyro_pitch_deg;
    float gyro_pitch_uncalibrated_deg;
    float fused_pitch_deg;
} Filter_HandleTypeDef;
void Filter_Init(Filter_HandleTypeDef *filter);
uint8_t Filter_Process(Filter_HandleTypeDef *filter,
                       const MPU6050_DataTypeDef *sample, float dt_s);
uint8_t Filter_IsCalibrating(const Filter_HandleTypeDef *filter);
#endif
