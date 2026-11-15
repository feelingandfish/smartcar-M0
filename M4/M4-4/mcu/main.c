/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : M4-4 上下位机协议 —— 主程序
  *
  * 结构：
  *   TIM4 10 ms 中断  → 编码器采样 + PID + PWM（沿用 M4-3 已验证的闭环）
  *                     → 每 5 次（50 ms）置一次上报标志
  *   USART1 RX 中断   → 只把字节塞进环形缓冲（uart.c）
  *   主循环           → 从缓冲取字节喂状态机（protocol.c）→ 执行指令 / 发状态帧
  *
  * 本程序【没有任何 while 等待串口】的地方：接收靠中断+环形缓冲，
  * 发送只在 50 ms 节点做一次短阻塞（20 B @115200 ≈ 1.7 ms）。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "ringbuf.h"
#include "protocol.h"
#include "uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_SAMPLE_MS   10U
#define STATUS_PERIOD_MS    50U                                   /* 题面要求 50 ms 上报 */
#define STATUS_TICKS        (STATUS_PERIOD_MS / ENCODER_SAMPLE_MS) /* = 5 次 10ms 中断 */

#define PID_PWM_LIMIT       500.0f
#define ENCODER_RPM_SIGN    (1.0f)    /* 沿用 M4-3 最终标定：正 PWM 对应正计数，保持正号 */

/* 速度环参数：M4-3 最终定稿 Kp=13 / Ki=1.0 / Kd=0.01（2.3 阶跃三组对比后的结论） */
#define SPEED_PID_KP        13.0f
#define SPEED_PID_KI        1.0f
#define SPEED_PID_KD        0.01f

#define CMD_RPM_LIMIT       120       /* 上位机指令安全上限，与 M4-3 的 TARGET 范围一致（0..120） */
#define CMD_TIMEOUT_MS      500U      /* 超过 500 ms 收不到 0x01 → 目标平滑归零（掉线安全停车） */
#define CMD_RAMP_STEP       5.0f      /* 超时停车时每 10 ms 减 5 r/min */

#define PITCH_X100          0         /* 本阶段未接 MPU6050，pitch 填 0（字段保留） */
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
RingBuf_t            g_rxbuf;                    /* uart.c 里 extern 使用 */
static ProtoParser_t g_parser;

static volatile float   actual_rpm   = 0.0f;
static volatile float   target_rpm   = 0.0f;     /* 上电静止，等上位机指令 */
static volatile int32_t pwm_out     = 0;
static volatile int16_t rpm_report   = 0;        /* 上报用的实测转速快照 */

static volatile uint16_t encoder_cnt_prev = 0;
static volatile uint32_t tick_ms          = 0;
static volatile uint16_t heartbeat        = 0;

static volatile uint8_t  status_tick     = 0;    /* 10 ms 中断计数 */
static volatile uint8_t  status_tx_flag  = 0;
static volatile uint32_t last_cmd_tick   = 0;    /* 最近一次收到速度指令的时刻 */

static uint8_t  last_speed_seq = 0;
static uint8_t  tx_frame[20];
static uint32_t last_led = 0;

static PID_t speed_pid;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
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
  uint8_t  b;
  uint16_t frame_len;
  int16_t  cmd;
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

  /* 协议栈：环形缓冲 + 解析器 + 中断接收 */
  RingBuf_Init(&g_rxbuf);
  Proto_Init(&g_parser);
  Uart_Start();

  last_cmd_tick = 0;
  HAL_TIM_Base_Start_IT(&htim4);     /* 10 ms 基准，同时承担 50 ms 上报节拍 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---- 1) 把手里的字节全部喂给状态机（不阻塞，取空即退出） ---- */
    while (RingBuf_Get(&g_rxbuf, &b) != 0u) {
        Proto_Feed(&g_parser, b);
    }

    /* ---- 2) 帧内超时检查（收到一半就断了 → 计 timeout 并回找帧头） ---- */
    Proto_CheckTimeout(&g_parser, HAL_GetTick());

    /* ---- 3) 0x03 清计数 ---- */
    if (g_parser.clear_req != 0u) {
        g_parser.clear_req     = 0u;
        g_parser.bad_frame_cnt = 0u;
        g_parser.timeout_cnt   = 0u;
        g_parser.overflow_cnt  = 0u;
        status_tx_flag         = 1u;    /* 立刻回一帧，让上位机马上看到归零 */
    }

    /* ---- 4) 收到新的 0x01 → 更新速度目标（限幅 + 记录时刻） ---- */
    if (g_parser.speed_cmd_seq != last_speed_seq) {
        last_speed_seq = g_parser.speed_cmd_seq;

        cmd = g_parser.cmd_left_rpm;
        if (cmd >  CMD_RPM_LIMIT) cmd =  CMD_RPM_LIMIT;
        if (cmd < -CMD_RPM_LIMIT) cmd = -CMD_RPM_LIMIT;

        target_rpm    = (float)cmd;
        last_cmd_tick = tick_ms;
    }

    /* ---- 5) 50 ms 到 → 组状态帧并发送 ---- */
    if (status_tx_flag != 0u) {
        status_tx_flag = 0u;
        heartbeat++;

        frame_len = Proto_BuildStatus(tx_frame,
                                      rpm_report,
                                      rpm_report,          /* 单电机方案：左右同速 */
                                      (int16_t)PITCH_X100,
                                      heartbeat,
                                      g_parser.bad_frame_cnt,
                                      g_parser.timeout_cnt,
                                      g_parser.overflow_cnt);

        HAL_UART_Transmit(&huart1, tx_frame, frame_len, 10);
    }

    /* ---- 6) PC13 心跳灯：500 ms 翻转，肉眼判断固件还在跑 ---- */
    if ((uint32_t)(tick_ms - last_led) >= 500U) {
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
 * TIM4 10 ms 周期：
 *   1. 编码器增量 → 实测转速 → PID → PWM（沿用 M4-3 的固定周期闭环）
 *   2. 累计 tick_ms，每 5 次（50 ms）置一次上报标志（主循环负责真正发送）
 *   3. 指令超时（500 ms 没收到 0x01）→ 目标平滑归零，避免拔线后电机持续飞转
 *
 * 注意：中断里【不做串口发送】，只置标志，保证 PID 周期不被 UART 阻塞拉长。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        uint16_t now_cnt = __HAL_TIM_GET_COUNTER(&htim2);
        int16_t  delta   = (int16_t)(now_cnt - encoder_cnt_prev);
        float    pid_output;
        int32_t  pid_pwm;

        encoder_cnt_prev = now_cnt;
        tick_ms += ENCODER_SAMPLE_MS;

        actual_rpm = ENCODER_RPM_SIGN * (float)delta * 6000.0f /
                     (float)ENC_COUNTS_PER_REV;

        rpm_report = (int16_t)(actual_rpm >= 0.0f
                               ? actual_rpm + 0.5f : actual_rpm - 0.5f);

        /* 掉线安全停车：长时间收不到速度指令，目标缓慢减到 0 */
        if ((uint32_t)(tick_ms - last_cmd_tick) > CMD_TIMEOUT_MS) {
            if (target_rpm > 0.0f) {
                target_rpm -= CMD_RAMP_STEP;
                if (target_rpm < 0.0f) target_rpm = 0.0f;
            } else if (target_rpm < 0.0f) {
                target_rpm += CMD_RAMP_STEP;
                if (target_rpm > 0.0f) target_rpm = 0.0f;
            }
        }

        pid_output = PID_Compute(&speed_pid, target_rpm, actual_rpm, 0.010f);
        pid_pwm    = (int32_t)(pid_output >= 0.0f
                               ? pid_output + 0.5f : pid_output - 0.5f);
        pwm_out    = pid_pwm;
        Motor_SetPWM(pid_pwm);

        /* 50 ms 上报节拍 */
        status_tick++;
        if (status_tick >= STATUS_TICKS) {
            status_tick    = 0u;
            status_tx_flag = 1u;
        }
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
  * @param  line: source line number
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
