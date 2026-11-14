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
#include <string.h>
#include "encoder.h"
#include "motor.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 启动默认参数；运行中可通过 USART1 命令修改 Kp、Ki、Kd。 */
#define ENCODER_SAMPLE_MS 10U
#define PID_TARGET_RPM    60.0f
#define PID_TARGET_RPM_MAX 120.0f /* 在线目标转速安全上限 */
#define PID_PWM_LIMIT     500.0f
#define ENCODER_RPM_SIGN  (1.0f) /* 本次采样：正PWM时delta_cnt为正，反馈保持正号 */

/* 以下是上电默认值；也可在运行中发送 PID,Kp,Ki,Kd 覆盖。 */
#define SPEED_PID_KP 13.0f
#define SPEED_PID_KI 1.0f
#define SPEED_PID_KD 0.01f
/* 在线调参保护范围；超过范围的串口命令会被拒绝。 */
#define PID_KP_MAX 50.0f
#define PID_KI_MAX 10.0f
#define PID_KD_MAX 1.0f
/* USART1 RX 中断只收字节，主循环解析命令；环形缓冲区大小取 2 的幂。 */
#define UART_RX_RING_SIZE 128U
#define UART_RX_RING_MASK (UART_RX_RING_SIZE - 1U)
#define PID_CMD_LINE_SIZE 48U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static volatile float    actual_rpm = 0.0f;
static volatile float    target_rpm = PID_TARGET_RPM;
static volatile int32_t  pwm_out = 0;
static volatile uint16_t encoder_cnt = 0;
static volatile int16_t  encoder_delta = 0;
static volatile uint32_t tick_ms = 0;
static volatile uint8_t  sample_flag = 0;
static char txbuf[96];
static uint32_t last_led = 0;
static PID_t speed_pid;
/* 在线调参接收缓冲：中断只写环形缓冲，PID 参数只在主循环更新。 */
static uint8_t uart_rx_byte;
static uint8_t uart_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t uart_rx_head = 0U;
static volatile uint16_t uart_rx_tail = 0U;
static volatile uint8_t uart_rx_overflow = 0U;
static char uart_cmd_line[PID_CMD_LINE_SIZE];
static uint16_t uart_cmd_len = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* 在线调参：主循环收完整行并解析，不在 10 ms 控制中断里做字符串处理。 */
static void ProcessUartRx(void);
static void HandleUartCommand(const char *line);
static void UartSendText(const char *text);
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
  uint32_t t_snap;
  uint16_t cnt_snap;
  int16_t delta_snap;
  float ac_snap;
  float target_snap;
  int32_t pw_snap;
  int len;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  Encoder_Init();
  Motor_Init();
  PID_Init(&speed_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
           PID_PWM_LIMIT, -PID_PWM_LIMIT);
  Motor_SetPWM(0);
  target_rpm = PID_TARGET_RPM;
  {
    static const uint8_t csv_header[] =
        "t_s,cnt,delta_cnt,target_rpm,actual_rpm,pwm\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)csv_header,
                      (uint16_t)(sizeof(csv_header) - 1U), 100);
  }
  /* 开启 USART1 单字节中断接收，避免主循环发送 CSV 时漏掉命令字节。 */
  if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U) != HAL_OK) {
    Error_Handler();
  }
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    /* 在线接收并处理调参命令；该工作在主循环，不阻塞定时器 PID 中断。 */
    ProcessUartRx();
    /* 上电即开始 60 r/min 闭环；串口调参命令不负责启动/停止电机。 */
    if (sample_flag) {
        __disable_irq();
        t_snap = tick_ms;
        cnt_snap = encoder_cnt;
        delta_snap = encoder_delta;
        ac_snap = actual_rpm;
        target_snap = target_rpm;
        pw_snap = pwm_out;
        sample_flag = 0U;
        __enable_irq();

        len = snprintf(txbuf, sizeof(txbuf), "%.2f,%u,%d,%.1f,%.1f,%ld\r\n",
                       (double)t_snap / 1000.0,
                       (unsigned int)cnt_snap,
                       (int)delta_snap,
                       (double)target_snap,
                       (double)ac_snap,
                       (long)pw_snap);
        HAL_UART_Transmit(&huart1, (uint8_t*)txbuf, (uint16_t)len, 50);
    }

    if ((tick_ms - last_led) >= 500U) {
        last_led = tick_ms;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
    /* USER CODE END 3 */
}
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;   /* TI1+TI2 4倍频 */
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;   /* 8MHz/(79+1)=100kHz; ARR=999 => 100Hz = 10ms */
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 * 在线调参命令协议（SSCOM 发送 ASCII，并勾选/使用 CRLF 换行）：
 *   PID,13,1,0.01  -> 同时设置 Kp、Ki、Kd。
 *   TARGET,20      -> 在线设置目标转速为 20 r/min（范围 0..120）。
 *   CSV             -> 重新输出 CSV 列名。
 *   HELP            -> 返回命令说明。
 * 参数限制：0<=Kp<=50，0<=Ki<=10，0<=Kd<=1；无效命令不会改当前参数。
 * 收到新参数后清积分/微分历史并对齐 prev_error，避免旧积分带入下一组。
 * # 开头为状态回执，不是数据行；CSV 命令输出的表头供分析脚本定位。
 */
static void UartSendText(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 50U);
}

static void HandleUartCommand(const char *line)
{
    float kp, ki, kd;
    char extra;
    char response[64];
    int fields;
    uint32_t primask;

    if (strcmp(line, "HELP") == 0) {
        UartSendText("#CMD,PID,<Kp>,<Ki>,<Kd>; TARGET,<rpm> (0..120); CSV\r\n");
        return;
    }

    if (strcmp(line, "CSV") == 0) {
        UartSendText("t_s,cnt,delta_cnt,target_rpm,actual_rpm,pwm\r\n");
        return;
    }
    if (strncmp(line, "TARGET,", 7U) == 0) {
        float target;

        fields = sscanf(line + 7, "%f %c", &target, &extra);
        if (fields != 1) {
            UartSendText("#TARGET,ERR,FORMAT; send TARGET,<rpm>\r\n");
            return;
        }
        if (!(target >= 0.0f && target <= PID_TARGET_RPM_MAX)) {
            UartSendText("#TARGET,ERR,RANGE; valid range 0..120 rpm\r\n");
            return;
        }

        /* 保留积分状态，只对齐误差历史，避免目标切换触发微分冲击。 */
        primask = __get_PRIMASK();
        __disable_irq();
        target_rpm = target;
        speed_pid.prev_error = target - actual_rpm;
        __set_PRIMASK(primask);

        (void)snprintf(response, sizeof(response), "#TARGET,OK,%.1f\r\n", (double)target);
        UartSendText(response);
        return;
    }
    if (strncmp(line, "PID,", 4U) != 0) {
        UartSendText("#PID,ERR,FORMAT; send PID,Kp,Ki,Kd\r\n");
        return;
    }

    /* 空格加 %c 用于拒绝数值后多余的非空白字符。 */
    fields = sscanf(line + 4, "%f,%f,%f %c", &kp, &ki, &kd, &extra);
    if (fields != 3) {
        UartSendText("#PID,ERR,FORMAT; example PID,13,1,0.01\r\n");
        return;
    }

    /* 范围判断同时会拒绝 NaN/无穷值；限幅可避免误发极端增益。 */
    if (!(kp >= 0.0f && kp <= PID_KP_MAX) ||
        !(ki >= 0.0f && ki <= PID_KI_MAX) ||
        !(kd >= 0.0f && kd <= PID_KD_MAX)) {
        UartSendText("#PID,ERR,RANGE; Kp=0..50 Ki=0..10 Kd=0..1\r\n");
        return;
    }

    /* 参数结构由 TIM4 中断读取；短暂关中断，确保三项一起切换。 */
    primask = __get_PRIMASK();
    __disable_irq();
    PID_SetParam(&speed_pid, kp, ki, kd);
    PID_Reset(&speed_pid);
    /* 让微分从当前误差起算，避免刚调参时产生人为微分冲击。 */
    speed_pid.prev_error = target_rpm - actual_rpm;
    __set_PRIMASK(primask);

    (void)snprintf(response, sizeof(response),
                   "#PID,OK,Kp=%.3f,Ki=%.3f,Kd=%.4f\r\n",
                   (double)kp, (double)ki, (double)kd);
    UartSendText(response);
}

static void ProcessUartRx(void)
{
    uint8_t ch;

    if (uart_rx_overflow != 0U) {
        /* 丢字节后丢弃半条命令，避免把残缺命令误当成有效参数。 */
        uart_rx_overflow = 0U;
        uart_rx_tail = uart_rx_head;
        uart_cmd_len = 0U;
        UartSendText("#PID,ERR,RX_OVERFLOW\r\n");
    }

    while (uart_rx_tail != uart_rx_head) {
        ch = uart_rx_ring[uart_rx_tail];
        uart_rx_tail = (uint16_t)((uart_rx_tail + 1U) & UART_RX_RING_MASK);

        if (ch == '\r' || ch == '\n') {
            if (uart_cmd_len > 0U) {
                uart_cmd_line[uart_cmd_len] = '\0';
                HandleUartCommand(uart_cmd_line);
                uart_cmd_len = 0U;
            }
        } else if (ch >= 32U && ch <= 126U) {
            if (uart_cmd_len < (PID_CMD_LINE_SIZE - 1U)) {
                uart_cmd_line[uart_cmd_len++] = (char)ch;
            } else {
                uart_cmd_len = 0U;
                UartSendText("#PID,ERR,CMD_TOO_LONG\r\n");
            }
        }
    }
}

/* USART1 接收中断只把字节放入环形缓冲并立即重启接收；不在中断里解析命令。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint16_t next = (uint16_t)((uart_rx_head + 1U) & UART_RX_RING_MASK);

        if (next != uart_rx_tail) {
            uart_rx_ring[uart_rx_head] = uart_rx_byte;
            uart_rx_head = next;
        } else {
            uart_rx_overflow = 1U;
        }

        (void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);
    }
}

/* USER CODE END 4 */
/* TIM4 每 10 ms 采样一次编码器，并在同一固定周期计算 PID。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        uint16_t now_cnt = __HAL_TIM_GET_COUNTER(&htim2);
        int16_t delta = (int16_t)(now_cnt - (uint16_t)encoder_cnt);
        float pid_output;
        int32_t pid_pwm;

        tick_ms += ENCODER_SAMPLE_MS;
        encoder_cnt = now_cnt;
        encoder_delta = delta;
        actual_rpm = ENCODER_RPM_SIGN * (float)delta * 6000.0f /
                     (float)ENC_COUNTS_PER_REV;

        pid_output = PID_Compute(&speed_pid, target_rpm, actual_rpm, 0.010f);
        pid_pwm = (int32_t)(pid_output >= 0.0f
                            ? pid_output + 0.5f : pid_output - 0.5f);
        pwm_out = pid_pwm;
        Motor_SetPWM(pid_pwm);
        sample_flag = 1U;
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
