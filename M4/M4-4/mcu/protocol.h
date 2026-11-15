#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ============ 协议常量（必须与 docs/protocol_v1.0.md 完全一致） ============ */
#define PROTO_HEAD0        0xAAu
#define PROTO_HEAD1        0x55u
#define PROTO_MAX_PAYLOAD  64u
#define PROTO_MAX_FRAME    (6u + PROTO_MAX_PAYLOAD)   /* 70 字节 */
#define PROTO_TIMEOUT_MS   50u

#define MSG_ID_SPEED       0x01u
#define MSG_ID_STATUS      0x02u
#define MSG_ID_CLEAR       0x03u

/* 状态机阶段 */
typedef enum {
    PS_WAIT_H1 = 0,      /* 等 0xAA */
    PS_WAIT_H2,          /* 等 0x55 */
    PS_WAIT_LEN,         /* 收长度 */
    PS_WAIT_ID,          /* 收消息 ID */
    PS_WAIT_PAYLOAD,     /* 收载荷 */
    PS_WAIT_CRC_L,       /* 收 CRC 低字节 */
    PS_WAIT_CRC_H        /* 收 CRC 高字节 */
} ProtoState_t;

typedef struct {
    /* --- 状态机 --- */
    ProtoState_t state;
    uint8_t      len;                        /* LEN 字段 */
    uint8_t      msg_id;
    uint8_t      payload[PROTO_MAX_PAYLOAD];
    uint8_t      idx;                        /* 载荷已收字节数 */
    uint16_t     crc_calc;                   /* 边收边算的 CRC */
    uint16_t     crc_rx;                     /* 帧里带的 CRC */

    /* --- 整帧原始字节，供 CRC 失败后"从起始+1"重扫 --- */
    uint8_t      frame_buf[PROTO_MAX_FRAME];
    uint8_t      frame_len;
    uint8_t      retry_flag;
    uint8_t      retry_start;

    /* --- 统计（uint16，溢出回绕，不饱和） --- */
    volatile uint16_t bad_frame_cnt;         /* CRC 失败次数 */
    volatile uint16_t timeout_cnt;           /* 帧内字节间隔超时次数 */
    volatile uint16_t overflow_cnt;          /* LEN > 64 次数 */

    /* --- 超时基准 --- */
    volatile uint32_t last_byte_ms;

    /* --- 解析结果，主循环取用 --- */
    volatile int16_t  cmd_left_rpm;          /* 0x01 左轮目标 */
    volatile int16_t  cmd_right_rpm;         /* 0x01 右轮目标 */
    volatile uint8_t  speed_cmd_seq;         /* 每收到一帧有效 0x01 就 +1 */
    volatile uint8_t  clear_req;             /* 收到 0x03 */
} ProtoParser_t;

/* 初始化 / 复位 */
void     Proto_Init(ProtoParser_t *p);

/* 喂一个字节（含 CRC 失败后的重同步） */
void     Proto_Feed(ProtoParser_t *p, uint8_t b);

/* 帧内超时检查，每轮主循环调用一次；now_ms 传 HAL_GetTick() */
void     Proto_CheckTimeout(ProtoParser_t *p, uint32_t now_ms);

/* CRC-16/MODBUS：poly 0xA001，init 0xFFFF，输入输出反射，结果不异或 */
uint16_t Proto_Crc16(const uint8_t *data, uint16_t n);

/* 组帧：状态回报（14 字节载荷，整帧 20 字节）。返回整帧长度。 */
uint16_t Proto_BuildStatus(uint8_t *out,
                           int16_t  left_rpm,
                           int16_t  right_rpm,
                           int16_t  pitch_x100,
                           uint16_t heartbeat,
                           uint16_t bad_frame_cnt,
                           uint16_t timeout_cnt,
                           uint16_t overflow_cnt);

/* 组帧：清计数（0 字节载荷，整帧 6 字节）。返回整帧长度。 */
uint16_t Proto_BuildClear(uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
