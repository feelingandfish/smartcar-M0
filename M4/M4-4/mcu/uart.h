#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * uart —— USART1 中断接收
 *
 * 中断里只做两件事：把字节塞进环形缓冲、重新挂上下一次接收。
 * 不做任何解析、不做 printf、不等待。这样即使连续来脏字节，
 * 中断也不会被拖长，主循环照常跑电机控制。
 */
void Uart_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */
