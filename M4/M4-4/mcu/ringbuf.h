#ifndef __RINGBUF_H
#define __RINGBUF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * ringbuf —— 单生产者/单消费者环形缓冲
 *
 * 生产者：USART1 RX 中断（只调用 RingBuf_Put，O(1)，不解析）
 * 消费者：主循环（只调用 RingBuf_Get，O(1)，不阻塞）
 *
 * 容量 1024 字节，取 2 的幂以便用掩码代替取模（更省周期）。
 * 写满时【覆盖最旧】：丢掉 tail 处的旧字节，保证最新数据一定能写进去，
 * 且永远不会越界、永远不会让中断阻塞。状态机本身能从错位重同步，
 * 因此丢旧数据不会导致永久失步。
 */
#define RINGBUF_SIZE   1024u          /* 必须为 2 的幂 */
#define RINGBUF_MASK   (RINGBUF_SIZE - 1u)

typedef struct {
    uint8_t           buf[RINGBUF_SIZE];
    volatile uint16_t head;       /* 写指针（仅中断写） */
    volatile uint16_t tail;       /* 读指针（仅主循环写） */
    volatile uint32_t drop_cnt;   /* 因写满而丢弃的最旧字节数（统计用） */
} RingBuf_t;

void     RingBuf_Init(RingBuf_t *rb);
void     RingBuf_Put(RingBuf_t *rb, uint8_t b);        /* 满则覆盖最旧，永不阻塞 */
uint8_t  RingBuf_Get(RingBuf_t *rb, uint8_t *b);       /* 1=取到，0=空 */
uint16_t RingBuf_Count(const RingBuf_t *rb);           /* 当前待读字节数 */

#ifdef __cplusplus
}
#endif

#endif /* __RINGBUF_H */
