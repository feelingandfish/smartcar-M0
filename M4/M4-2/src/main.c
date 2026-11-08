/* M4-2 application main: generated peripheral setup is kept below. */
#include "main.h"
#include "encoder.h"
#include "mpu6050.h"
#include "filter.h"
#include <stdio.h>

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

static MPU6050_HandleTypeDef g_mpu6050; /* callbacks and main share this state */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

#define IMU_SAMPLE_PERIOD_MS       10U
#define ENCODER_SAMPLE_PERIOD_MS   1000U
#define UART_OUTPUT_PERIOD_MS      100U
#define DRIFT_TEST_DURATION_MS   60000U
#define DRIFT_LOG_PERIOD_MS      1000U

int main(void)
{
    Encoder_HandleTypeDef encoder = {0};
    Filter_HandleTypeDef filter;
    MPU6050_DataTypeDef sample;
    char uartBuf[256];
    int len;
    uint32_t last_imu_tick;
    uint32_t last_imu_request_tick;
    uint32_t last_uart_output_tick;
    uint32_t fusion_start_tick;
    uint32_t drift_start_tick = 0U;
    uint32_t last_drift_log_tick = 0U;
    uint8_t drift_test_started = 0U;
    uint8_t drift_test_done = 0U;
    HAL_StatusTypeDef i2c_result;
    uint8_t who_am_i = 0U;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();

    /* 启动编码器模块，并建立本次计数和时间基准。 */
    Encoder_Init(&encoder, &htim2, HAL_GetTick());
    (void)Encoder_Start(&encoder, &htim2);

    /* MPU6050模块负责WHO_AM_I、初始化寄存器和非阻塞采样。 */
    MPU6050_InitHandle(&g_mpu6050, &hi2c1);
    Filter_Init(&filter);
    i2c_result = MPU6050_ReadWhoAmI(&g_mpu6050, &who_am_i);
    if (i2c_result == HAL_OK)
    {
        len = snprintf(uartBuf, sizeof(uartBuf), "MPU6050 WHO_AM_I=0x%02X\r\n", who_am_i);
    }
    else
    {
        len = snprintf(uartBuf, sizeof(uartBuf), "MPU6050 WHO_AM_I read failed, HAL=%d\r\n", (int)i2c_result);
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, (uint16_t)len, 100U);

    if ((i2c_result == HAL_OK) && (who_am_i == 0x68U))
    {
        i2c_result = MPU6050_Configure(&g_mpu6050);
        if (i2c_result == HAL_OK)
            len = snprintf(uartBuf, sizeof(uartBuf), "MPU6050 init OK: awake, accel +/-2g, gyro +/-250dps\r\n");
        else
            len = snprintf(uartBuf, sizeof(uartBuf), "MPU6050 config write failed, HAL=%d\r\n", (int)i2c_result);
        HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, (uint16_t)len, 100U);
    }

    last_imu_tick = HAL_GetTick();
    last_imu_request_tick = last_imu_tick;
    encoder.previous_tick = last_imu_tick;
    fusion_start_tick = last_imu_tick;
    last_uart_output_tick = last_imu_tick;
    len = snprintf(uartBuf, sizeof(uartBuf),
                   "Keep MPU6050 still: calibrating gyro bias (%u samples)\r\n",
                   (unsigned int)FILTER_GYRO_CALIBRATION_SAMPLES);
    HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, (uint16_t)len, 100U);

    while (1)
    {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed_ms;

        /* 编码器每秒计算一次差值和转速，窗口使用实际经过时间。 */
        if ((now - encoder.previous_tick) >= ENCODER_SAMPLE_PERIOD_MS)
        {
            if (Encoder_Update(&encoder, &htim2, now, &elapsed_ms) != 0U)
            {
                if ((drift_test_started == 0U) || (drift_test_done != 0U))
                {
                    len = snprintf(uartBuf, sizeof(uartBuf), "CNT:%d RPM:%.1f\r\n",
                                   (int)encoder.delta_count, encoder.rpm);
                    HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, (uint16_t)len, 100U);
                }
            }
        }

        /* 10 ms发起一次异步I2C读取；实际解析由主循环在完成标志后执行。 */
        if (((now - last_imu_request_tick) >= IMU_SAMPLE_PERIOD_MS) &&
            (g_mpu6050.busy == 0U) && (g_mpu6050.sample_ready == 0U))
        {
            last_imu_request_tick = now;
            (void)MPU6050_RequestSample(&g_mpu6050);
        }

        if (MPU6050_TakeSample(&g_mpu6050, &sample) != 0U)
        {
            uint8_t was_calibrated = filter.calibrated;
            uint8_t was_angle_initialized = filter.angle_initialized;
            float dt_s;
            float gx_corrected_dps;
            float gy_corrected_dps;
            float gz_corrected_dps;

            now = HAL_GetTick();
            elapsed_ms = now - last_imu_tick;
            dt_s = (float)elapsed_ms / 1000.0f;
            last_imu_tick = now;
            (void)Filter_Process(&filter, &sample, dt_s);

            if ((was_calibrated == 0U) && (filter.calibrated != 0U))
            {
                len = snprintf(uartBuf, sizeof(uartBuf),
                    "Gyro bias calibrated [dps]: X=%+.3f Y=%+.3f Z=%+.3f\r\n",
                    filter.gyro_bias_x_dps, filter.gyro_bias_y_dps, filter.gyro_bias_z_dps);
                HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, (uint16_t)len, 100U);
            }

            gx_corrected_dps = sample.gx_dps - filter.gyro_bias_x_dps;
            gy_corrected_dps = sample.gy_dps - filter.gyro_bias_y_dps;
            gz_corrected_dps = sample.gz_dps - filter.gyro_bias_z_dps;

            /* 零偏校准后的首个样本提供共同角度初值，并启动60秒漂移记录。 */
            if ((was_angle_initialized == 0U) && (filter.angle_initialized != 0U))
            {
                drift_start_tick = now;
                last_drift_log_tick = now;
                drift_test_started = 1U;
                len = snprintf(uartBuf, sizeof(uartBuf),
                    "DRIFT_CSV_BEGIN\r\n"
                    "t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,gyro_bias_compensated_deg,gyro_uncompensated_deg,fused_pitch_deg\r\n"
                    "0.00,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f\r\n",
                    sample.ax_g, sample.ay_g, sample.az_g,
                    gx_corrected_dps, gy_corrected_dps, gz_corrected_dps,
                    filter.gyro_pitch_deg, filter.gyro_pitch_uncalibrated_deg,
                    filter.fused_pitch_deg);
                HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, (uint16_t)len, 100U);
            }

            /* 静置漂移CSV每秒一行；测试时抑制CNT文本，避免混入CSV。 */
            if ((drift_test_started != 0U) && (drift_test_done == 0U) &&
                ((now - last_drift_log_tick) >= DRIFT_LOG_PERIOD_MS))
            {
                float drift_elapsed_s = (float)(now - drift_start_tick) / 1000.0f;
                last_drift_log_tick = now;
                len = snprintf(uartBuf, sizeof(uartBuf),
                    "%.2f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f,%+.3f\r\n",
                    drift_elapsed_s, sample.ax_g, sample.ay_g, sample.az_g,
                    gx_corrected_dps, gy_corrected_dps, gz_corrected_dps,
                    filter.gyro_pitch_deg, filter.gyro_pitch_uncalibrated_deg,
                    filter.fused_pitch_deg);
                HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, (uint16_t)len, 100U);
                if ((now - drift_start_tick) >= DRIFT_TEST_DURATION_MS)
                {
                    drift_test_done = 1U;
                    len = snprintf(uartBuf, sizeof(uartBuf), "DRIFT_CSV_END\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, (uint16_t)len, 100U);
                }
            }

            if ((filter.calibrated != 0U) &&
                ((drift_test_started == 0U) || (drift_test_done != 0U)) &&
                ((now - last_uart_output_tick) >= UART_OUTPUT_PERIOD_MS))
            {
                last_uart_output_tick = now;
                len = snprintf(uartBuf, sizeof(uartBuf),
                    "T=%.2f ACC[g]:X=%+.3f Y=%+.3f Z=%+.3f GYRO[dps]:X=%+.1f Y=%+.1f Z=%+.1f P_ACC=%+.1f P_GYRO=%+.1f PITCH=%+.1f\r\n",
                    (float)(now - fusion_start_tick) / 1000.0f,
                    sample.ax_g, sample.ay_g, sample.az_g,
                    gx_corrected_dps, gy_corrected_dps, gz_corrected_dps,
                    filter.acc_pitch_deg, filter.gyro_pitch_deg, filter.fused_pitch_deg);
                HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, (uint16_t)len, 100U);
            }
        }

        if (Filter_IsCalibrating(&filter) &&
            ((now - last_uart_output_tick) >= UART_OUTPUT_PERIOD_MS))
        {
            last_uart_output_tick = now;
            len = snprintf(uartBuf, sizeof(uartBuf), "Gyro calibration %u/%u: keep sensor still\r\n",
                (unsigned int)filter.calibration_count,
                (unsigned int)FILTER_GYRO_CALIBRATION_SAMPLES);
            HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, (uint16_t)len, 100U);
        }

        if (MPU6050_TakeError(&g_mpu6050) != 0U)
        {
            len = snprintf(uartBuf, sizeof(uartBuf),
                           "MPU6050 nonblocking I2C read error; retrying\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, (uint16_t)len, 100U);
        }
        HAL_Delay(1U);
    }
}

/* HAL中断回调只设置模块状态标志，不在中断中处理数据。 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    MPU6050_OnReceiveComplete(&g_mpu6050, hi2c);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    MPU6050_OnError(&g_mpu6050, hi2c);
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

