#include "uart.h"
#include "ringbuf.h"

extern UART_HandleTypeDef huart1;
extern RingBuf_t          g_rxbuf;      /* 定义在 main.c */

static uint8_t s_rx_byte;

/* 启动第一次接收（之后由回调自己续挂） */
void Uart_Start(void)
{
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
}

/*
 * 接收完成回调：O(1) 入队后立刻重新挂接收。
 * 这是"中断只搬运"的关键——解析全部交给主循环的状态机。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        RingBuf_Put(&g_rxbuf, s_rx_byte);
        HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
    }
}

/*
 * 错误回调：串口出错（最常见是 ORE 溢出、以及拔插线时的噪声帧错误）。
 * HAL 在出错后会【停止接收】，如果不管，表现为"串口从此再无数据"。
 * 这里清掉错误标志并重新挂接收，保证拔线重插后能自己恢复，不需要复位。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
    }
}
