#ifndef MPU6050_H
#define MPU6050_H
#include "main.h"
#include <stdint.h>
#define MPU6050_ADDRESS_7BIT        0x68U
#define MPU6050_ADDRESS_HAL         (MPU6050_ADDRESS_7BIT << 1U)
#define MPU6050_REG_WHO_AM_I        0x75U
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU
#define MPU6050_RAW_DATA_LENGTH     14U
#define MPU6050_GYRO_LSB_PER_DPS    131.0f  /* ±250 °/s */
#define MPU6050_ACCEL_LSB_PER_G     16384.0f /* ±2 g */
typedef struct {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
} MPU6050_DataTypeDef;
typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t raw[MPU6050_RAW_DATA_LENGTH];
    volatile uint8_t busy;
    volatile uint8_t sample_ready;
    volatile uint8_t error;
} MPU6050_HandleTypeDef;
void MPU6050_InitHandle(MPU6050_HandleTypeDef *imu, I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef MPU6050_ReadWhoAmI(MPU6050_HandleTypeDef *imu, uint8_t *who_am_i);
HAL_StatusTypeDef MPU6050_Configure(MPU6050_HandleTypeDef *imu);
HAL_StatusTypeDef MPU6050_RequestSample(MPU6050_HandleTypeDef *imu);
uint8_t MPU6050_TakeSample(MPU6050_HandleTypeDef *imu, MPU6050_DataTypeDef *data);
uint8_t MPU6050_TakeError(MPU6050_HandleTypeDef *imu);
void MPU6050_OnReceiveComplete(MPU6050_HandleTypeDef *imu, I2C_HandleTypeDef *i2c);
void MPU6050_OnError(MPU6050_HandleTypeDef *imu, I2C_HandleTypeDef *i2c);
#endif
