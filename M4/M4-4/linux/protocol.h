#ifndef CARLINK_PROTOCOL_H
#define CARLINK_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============ 与 MCU 端 mcu/protocol.h 必须完全一致 ============ */
#define CARLINK_HEAD0        0xAAu
#define CARLINK_HEAD1        0x55u
#define CARLINK_MAX_PAYLOAD  64u
#define CARLINK_MAX_FRAME    (6u + CARLINK_MAX_PAYLOAD)
#define CARLINK_TIMEOUT_MS   50u

#define CARLINK_MSG_SPEED    0x01u
#define CARLINK_MSG_STATUS   0x02u
#define CARLINK_MSG_CLEAR    0x03u

typedef enum {
    CL_WAIT_H1 = 0,
    CL_WAIT_H2,
    CL_WAIT_LEN,
    CL_WAIT_ID,
    CL_WAIT_PAYLOAD,
    CL_WAIT_CRC_L,
    CL_WAIT_CRC_H
} cl_state_t;

typedef struct {
    /* --- 状态机 --- */
    cl_state_t  state;
    uint8_t     len;
    uint8_t     msg_id;
    uint8_t     payload[CARLINK_MAX_PAYLOAD];
    uint8_t     idx;
    uint16_t    crc_calc;
    uint16_t    crc_rx;

    /* --- 重同步用 --- */
    uint8_t     frame_buf[CARLINK_MAX_FRAME];
    uint8_t     frame_len;
    uint8_t     retry_flag;
    uint8_t     retry_start;

    /* --- 统计 --- */
    uint32_t    bad_frame_cnt;      /* 本工具侧统计（对方也会报一份） */
    uint32_t    timeout_cnt;
    uint32_t    overflow_cnt;

    uint32_t    last_byte_ms;

    /* --- 解析出的状态帧 --- */
    int16_t     st_left_rpm;
    int16_t     st_right_rpm;
    int16_t     st_pitch_x100;
    uint16_t    st_heartbeat;
    uint16_t    st_bad;
    uint16_t    st_timeout;
    uint16_t    st_overflow;
    uint32_t    st_seq;             /* 每收到一帧有效 0x02 就 +1 */
} cl_parser_t;

void     carlink_init(cl_parser_t *p, uint32_t now_ms);
void     carlink_feed(cl_parser_t *p, uint8_t b, uint32_t now_ms);
void     carlink_check_timeout(cl_parser_t *p, uint32_t now_ms);
uint16_t carlink_crc16(const uint8_t *data, uint16_t n);

uint16_t carlink_build_speed(uint8_t *out, int16_t left_rpm, int16_t right_rpm);
uint16_t carlink_build_status(uint8_t *out, int16_t left, int16_t right, int16_t pitch_x100,
                              uint16_t hb, uint16_t bad, uint16_t tmo, uint16_t ovf);
uint16_t carlink_build_clear(uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CARLINK_PROTOCOL_H */
