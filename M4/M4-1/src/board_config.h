#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include "main.h"

/* ===== M4-1 引脚定义集中处：现场换板只改这里 ===== */
#define LED_PORT        GPIOC
#define LED_PIN         GPIO_PIN_13
#define LED_ACTIVE_LOW  1        /* 1=低电平点亮（板载LED） */

#define KEY_PORT        GPIOA
#define KEY_PIN         GPIO_PIN_0
#define KEY_ACTIVE_LOW  1        /* 1=按下接地 */

/* ===== 串口与心跳参数 ===== */
#define UART_BAUD       115200
#define HEARTBEAT_MS    500

/* ===== 按键消抖 ===== */
#define KEY_DEBOUNCE_MS 20

#endif /* __BOARD_CONFIG_H */
