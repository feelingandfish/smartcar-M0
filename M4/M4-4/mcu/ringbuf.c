#include "ringbuf.h"

void RingBuf_Init(RingBuf_t *rb)
{
    rb->head     = 0u;
    rb->tail     = 0u;
    rb->drop_cnt = 0u;
}

/*
 * 写入一个字节。调用者可能是中断，因此这里【不做任何等待】：
 * 缓冲满时把最旧的字节挤掉（tail 前移一格），然后照常写入。
 */
void RingBuf_Put(RingBuf_t *rb, uint8_t b)
{
    uint16_t next = (uint16_t)((rb->head + 1u) & RINGBUF_MASK);

    if (next == rb->tail) {
        /* 已满：丢弃最旧一个字节，tail 前进 */
        rb->tail = (uint16_t)((rb->tail + 1u) & RINGBUF_MASK);
        rb->drop_cnt++;
    }

    rb->buf[rb->head] = b;
    rb->head = next;
}

uint8_t RingBuf_Get(RingBuf_t *rb, uint8_t *b)
{
    if (rb->tail == rb->head) {
        return 0u;                       /* 空 */
    }
    *b = rb->buf[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1u) & RINGBUF_MASK);
    return 1u;
}

uint16_t RingBuf_Count(const RingBuf_t *rb)
{
    return (uint16_t)((rb->head - rb->tail) & RINGBUF_MASK);
}
