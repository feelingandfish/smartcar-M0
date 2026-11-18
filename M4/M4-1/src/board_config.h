#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include "main.h"

/* ===== M4-1 引脚定义集中处：现场换板只改这里 ===== */
#define LED_PORT        GPIOC
#define LED_PIN         GPIO_PIN_13
#define LED_ACTIVE_LOW  1        /* 1=低电平点亮（板载LED） */
#define LED_PORT_CLK_ENABLE()   __HAL_RCC_GPIOC_CLK_ENABLE()

#define KEY_PORT        GPIOA
#define KEY_PIN         GPIO_PIN_0
#define KEY_ACTIVE_LOW  1        /* 1=按下接地 */
#define KEY_PORT_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()

/* ===== 下面这段由上面的“极性”自动推出，换板时不用改 ===== */
#if LED_ACTIVE_LOW
#define LED_INIT_LEVEL   GPIO_PIN_RESET
#else
#define LED_INIT_LEVEL   GPIO_PIN_SET
#endif

#if KEY_ACTIVE_LOW
#define KEY_PULL         GPIO_PULLUP
#define KEY_IS_PRESSED() (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_RESET)
#else
#define KEY_PULL         GPIO_PULLDOWN
#define KEY_IS_PRESSED() (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_SET)
#endif

/* ===== 串口与心跳参数 ===== */
#define UART_BAUD       115200
#define HEARTBEAT_MS    500

/* ===== 按键消抖 ===== */
#define KEY_DEBOUNCE_MS 20

#endif /* __BOARD_CONFIG_H */
