/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 每圈计数(4倍频) = 磁环每圈7脉冲 × 4倍频 × 减速比50 = 1400，需实测校准 */
#define ENC_COUNTS_PER_REV  1385

/* MPU6050 姿态融合参数：50Hz 采样（20ms 一次），互补滤波系数先取常用起始值 */
#define IMU_SAMPLE_PERIOD_MS       20U
#define ENCODER_SAMPLE_PERIOD_MS   1000U
#define UART_OUTPUT_PERIOD_MS      100U   /* 串口10Hz显示；内部IMU融合仍保持50Hz */
#define COMPLEMENTARY_ALPHA        0.98f
#define RAD_TO_DEG                 57.2957795f
#define GYRO_CALIBRATION_SAMPLES  100U  /* 启动时采集100个静止样本估计零偏 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint32_t lastCnt = 0;
static char uartBuf[256]; /* 新增俯仰角字段后，串口输出行需要更大的缓存 */
/* MPU6050 连续读取缓存：0x3B 起共 14 字节，包含加速度、温度、角速度 */
static uint8_t mpu_raw[14];
/* I2C中断读取状态：中断回调只置标志，数据解析留在主循环 */
static volatile uint8_t imu_i2c_busy = 0U;
static volatile uint8_t imu_sample_ready = 0U;
static volatile uint8_t imu_i2c_error = 0U;
static int16_t ax_raw, ay_raw, az_raw;
static int16_t gx_raw, gy_raw, gz_raw;
static float ax_g, ay_g, az_g;
static float gx_dps, gy_dps, gz_dps;
static HAL_StatusTypeDef mpu_read_status;
/* 启动时由静止样本自动求陀螺仪零偏，避免把某一块传感器的偏差写死 */
static uint8_t gyro_calibrating = 1U;
static uint16_t gyro_calibration_count = 0U;
static int32_t gyro_sum_x = 0;
static int32_t gyro_sum_y = 0;
static int32_t gyro_sum_z = 0;
static float gyro_bias_x_dps = 0.0f;
static float gyro_bias_y_dps = 0.0f;
static float gyro_bias_z_dps = 0.0f;
static int16_t delta;
static float rpm;
static int len;
static uint16_t cnt;
/* 用独立时钟分别安排 IMU 快采样和编码器 1 秒测速，彼此不影响 */
static uint32_t last_imu_tick = 0;
static uint32_t last_imu_request_tick = 0;
static uint32_t last_encoder_tick = 0;
static uint32_t fusion_start_tick = 0;
static uint32_t last_uart_output_tick = 0;
/* 俯仰角：加速度计角、纯陀螺积分角、互补滤波融合角，单位均为度 */
static float pitch_acc_deg = 0.0f;
static float pitch_gyro_deg = 0.0f;
static float pitch_fused_deg = 0.0f;
static uint8_t pitch_initialized = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); /* 启动编码器计数 */

  /* 启动时检查并初始化 MPU6050：模块 AD0 接地时地址为 0x68 */
  {
    uint8_t who_am_i = 0;
    uint8_t reg_value = 0;
    HAL_StatusTypeDef i2c_result;

    /* 读取 WHO_AM_I 寄存器 0x75，正常 MPU6050 应返回 0x68 */
    i2c_result = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x68 << 1), 0x75,
                                  I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100);
    if (i2c_result == HAL_OK)
    {
      len = snprintf(uartBuf, sizeof(uartBuf),
                     "MPU6050 WHO_AM_I=0x%02X\r\n", who_am_i);
    }
    else
    {
      len = snprintf(uartBuf, sizeof(uartBuf),
                     "MPU6050 WHO_AM_I read failed, HAL=%d\r\n", i2c_result);
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, (uint16_t)len, 100);

    /* 只有身份读取成功且芯片应答 0x68，才继续配置 */
    if ((i2c_result == HAL_OK) && (who_am_i == 0x68))
    {
      /* PWR_MGMT_1(0x6B)=0：清除 SLEEP 位，唤醒传感器 */
      i2c_result = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(0x68 << 1), 0x6B,
                                     I2C_MEMADD_SIZE_8BIT, &reg_value, 1, 100);

      /* GYRO_CONFIG(0x1B)=0：陀螺仪量程设为 ±250°/s */
      if (i2c_result == HAL_OK)
      {
        i2c_result = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(0x68 << 1), 0x1B,
                                       I2C_MEMADD_SIZE_8BIT, &reg_value, 1, 100);
      }

      /* ACCEL_CONFIG(0x1C)=0：加速度量程设为 ±2g */
      if (i2c_result == HAL_OK)
      {
        i2c_result = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(0x68 << 1), 0x1C,
                                       I2C_MEMADD_SIZE_8BIT, &reg_value, 1, 100);
      }

      /* 把初始化结果也从串口发出，便于确认三个寄存器都写成功 */
      if (i2c_result == HAL_OK)
      {
        len = snprintf(uartBuf, sizeof(uartBuf),
                       "MPU6050 init OK: awake, accel +/-2g, gyro +/-250dps\r\n");
      }
      else
      {
        len = snprintf(uartBuf, sizeof(uartBuf),
                       "MPU6050 config write failed, HAL=%d\r\n", i2c_result);
      }
      HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, (uint16_t)len, 100);
    }
  }
  /* 从同一时刻起算两个独立时间窗：IMU 20ms，编码器 1000ms */
  last_imu_tick = HAL_GetTick();
  last_imu_request_tick = last_imu_tick;
  last_encoder_tick = last_imu_tick;
  fusion_start_tick = last_imu_tick;
  last_uart_output_tick = last_imu_tick;
  len = snprintf(uartBuf, sizeof(uartBuf),
                 "Keep MPU6050 still: calibrating gyro bias (%u samples)\r\n",
                 (unsigned int)GYRO_CALIBRATION_SAMPLES);
  HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, (uint16_t)len, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now;
    uint32_t elapsed_ms;
    float dt_s;
    float encoder_dt_s;
    float pitch_acc_absolute_deg;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    now = HAL_GetTick();

    /* 编码器仍然每约1秒计算一次；这样不会把原来的测速窗口改成20ms */
    elapsed_ms = now - last_encoder_tick;
    if (elapsed_ms >= ENCODER_SAMPLE_PERIOD_MS)
    {
      encoder_dt_s = (float)elapsed_ms / 1000.0f;
      cnt = __HAL_TIM_GET_COUNTER(&htim2);           /* 读取当前硬件计数器 */
      delta = (int16_t)(cnt - lastCnt);              /* 这段时间内新增的脉冲数 */
      lastCnt = cnt;
      last_encoder_tick = now;
      /* RPM = 脉冲数/每圈计数 × 60秒/实际采样秒数 */
      rpm = ((float)delta / (float)ENC_COUNTS_PER_REV) * (60.0f / encoder_dt_s);
      len = snprintf(uartBuf, sizeof(uartBuf), "CNT:%d RPM:%.1f\r\n", delta, rpm);
      HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, (uint16_t)len, 100);
    }

    /* 主循环只发起非阻塞I2C请求；传感器读完后由中断回调通知 */
    elapsed_ms = now - last_imu_request_tick;
    if ((elapsed_ms >= IMU_SAMPLE_PERIOD_MS) && (imu_i2c_busy == 0U))
    {
      last_imu_request_tick = now;
      mpu_read_status = HAL_I2C_Mem_Read_IT(&hi2c1, (uint16_t)(0x68 << 1), 0x3B,
                                            I2C_MEMADD_SIZE_8BIT, mpu_raw, 14);
      if (mpu_read_status == HAL_OK)
      {
        imu_i2c_busy = 1U;
      }
      else if (mpu_read_status != HAL_BUSY)
      {
        /* 未能启动本次请求时记录错误；下个周期继续尝试 */
        imu_i2c_error = 1U;
      }
    }

    /* I2C接收完成后，主循环解析14字节并执行单位换算、校准和融合 */
    if (imu_sample_ready != 0U)
    {
      imu_sample_ready = 0U;
      imu_i2c_busy = 0U;
      mpu_read_status = HAL_OK;
      now = HAL_GetTick();
      elapsed_ms = now - last_imu_tick;
      dt_s = (float)elapsed_ms / 1000.0f;
      last_imu_tick = now;

      /* 高字节在前，将每两字节合并成有正负号的16位原始值 */
      ax_raw = (int16_t)((mpu_raw[0] << 8) | mpu_raw[1]);
      ay_raw = (int16_t)((mpu_raw[2] << 8) | mpu_raw[3]);
      az_raw = (int16_t)((mpu_raw[4] << 8) | mpu_raw[5]);
      gx_raw = (int16_t)((mpu_raw[8] << 8) | mpu_raw[9]);
      gy_raw = (int16_t)((mpu_raw[10] << 8) | mpu_raw[11]);
      gz_raw = (int16_t)((mpu_raw[12] << 8) | mpu_raw[13]);

      /* ±2g量程下16384 LSB/g；±250°/s量程下131 LSB/(°/s) */
      ax_g = (float)ax_raw / 16384.0f;
      ay_g = (float)ay_raw / 16384.0f;
      az_g = (float)az_raw / 16384.0f;
      gx_dps = (float)gx_raw / 131.0f;
      gy_dps = (float)gy_raw / 131.0f;
      gz_dps = (float)gz_raw / 131.0f;

      if (gyro_calibrating != 0U)
      {
        /* 静止时平均100个原始角速度样本，得到本次上电的零偏 */
        gyro_sum_x += gx_raw;
        gyro_sum_y += gy_raw;
        gyro_sum_z += gz_raw;
        gyro_calibration_count++;
        if (gyro_calibration_count >= GYRO_CALIBRATION_SAMPLES)
        {
          gyro_bias_x_dps = ((float)gyro_sum_x / (float)gyro_calibration_count) / 131.0f;
          gyro_bias_y_dps = ((float)gyro_sum_y / (float)gyro_calibration_count) / 131.0f;
          gyro_bias_z_dps = ((float)gyro_sum_z / (float)gyro_calibration_count) / 131.0f;
          gyro_calibrating = 0U;
          len = snprintf(uartBuf, sizeof(uartBuf),
                         "Gyro bias calibrated [dps]: X=%+.3f Y=%+.3f Z=%+.3f\r\n",
                         gyro_bias_x_dps, gyro_bias_y_dps, gyro_bias_z_dps);
          HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, (uint16_t)len, 100);
        }
      }
      else
      {
        /* 用启动时实测得到的零偏修正当前陀螺仪角速度 */
        gx_dps -= gyro_bias_x_dps;
        gy_dps -= gyro_bias_y_dps;
        gz_dps -= gyro_bias_z_dps;

        /* 俯仰绕Y轴：由X轴重力分量估角，Y/Z合成量作分母。 */
        pitch_acc_absolute_deg = atan2f(-ax_g,
                                        sqrtf(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;
        if (pitch_initialized == 0U)
        {
          pitch_acc_deg = pitch_acc_absolute_deg;
          pitch_gyro_deg = pitch_acc_absolute_deg;
          pitch_fused_deg = pitch_acc_absolute_deg;
          pitch_initialized = 1U;
        }
        else
        {
          pitch_acc_deg = pitch_acc_absolute_deg;
          pitch_gyro_deg += gy_dps * dt_s;
          pitch_fused_deg = COMPLEMENTARY_ALPHA *
                            (pitch_fused_deg + gy_dps * dt_s) +
                            (1.0f - COMPLEMENTARY_ALPHA) * pitch_acc_deg;
        }
      }

      /* 串口每100ms输出一次，减少刷屏；上面的读取和融合仍每20ms执行 */
      if ((now - last_uart_output_tick) >= UART_OUTPUT_PERIOD_MS)
      {
        last_uart_output_tick = now;
        if (gyro_calibrating != 0U)
        {
          len = snprintf(uartBuf, sizeof(uartBuf),
                         "Gyro calibration %u/%u: keep sensor still\r\n",
                         (unsigned int)gyro_calibration_count,
                         (unsigned int)GYRO_CALIBRATION_SAMPLES);
        }
        else
        {
          len = snprintf(uartBuf, sizeof(uartBuf),
                         "T=%.2f ACC[g]:X=%+.3f Y=%+.3f Z=%+.3f GYRO[dps]:X=%+.1f Y=%+.1f Z=%+.1f P_ACC=%+.1f P_GYRO=%+.1f PITCH=%+.1f\r\n",
                         (float)(now - fusion_start_tick) / 1000.0f,
                         ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps,
                         pitch_acc_deg, pitch_gyro_deg, pitch_fused_deg);
        }
        HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, (uint16_t)len, 100);
      }
    }

    if (imu_i2c_error != 0U)
    {
      imu_i2c_error = 0U;
      imu_i2c_busy = 0U;
      len = snprintf(uartBuf, sizeof(uartBuf),
                     "MPU6050 nonblocking I2C read error; retrying\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, (uint16_t)len, 100);
    }

    HAL_Delay(1); /* 让主循环短暂等待，实际采样间隔由上面的毫秒时钟控制 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12; /* TI1+TI2 4倍频 */
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* I2C中断接收完成：只通知主循环，避免在中断里做复杂计算或串口打印 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    imu_sample_ready = 1U;
  }
}

/* I2C发生错误时通知主循环恢复状态并重试 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    imu_i2c_busy = 0U;
    imu_i2c_error = 1U;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
