/**
 ******************************************************************************
 * protocol.c —— CarLink v1.0 解析/组帧（Linux 端，与 MCU 端逻辑对称）
 *
 * 与 mcu/protocol.c 使用同一套规则：
 *   帧头 AA 55 / LEN<=64 / CRC-16/MODBUS 小端 / 帧内 50ms 超时 /
 *   CRC 失败从起始+1 重扫。
 ******************************************************************************
 */
#include "protocol.h"
#include <string.h>

static void cl_sm_reset(cl_parser_t *p)
{
    p->state     = CL_WAIT_H1;
    p->len       = 0u;
    p->msg_id    = 0u;
    p->idx       = 0u;
    p->crc_calc  = 0xFFFFu;
    p->crc_rx    = 0u;
    p->frame_len = 0u;
}

static void cl_crc_update(cl_parser_t *p, uint8_t b)
{
    uint8_t i;

    p->crc_calc ^= b;
    for (i = 0u; i < 8u; i++) {
        if (p->crc_calc & 1u) {
            p->crc_calc = (uint16_t)((p->crc_calc >> 1) ^ 0xA001u);
        } else {
            p->crc_calc = (uint16_t)(p->crc_calc >> 1);
        }
    }
}

static void cl_record(cl_parser_t *p, uint8_t b)
{
    if (p->frame_len < CARLINK_MAX_FRAME) {
        p->frame_buf[p->frame_len] = b;
        p->frame_len++;
    }
}

static void cl_dispatch(cl_parser_t *p)
{
    int i;

    switch (p->msg_id) {
    case CARLINK_MSG_STATUS:
        if (p->len == 14u) {
            /* 小端解析，注意 int16 的补码由 (int16_t) 强转天然完成 */
            p->st_left_rpm  = (int16_t)((uint16_t)p->payload[0] |
                                       ((uint16_t)p->payload[1] << 8));
            p->st_right_rpm = (int16_t)((uint16_t)p->payload[2] |
                                       ((uint16_t)p->payload[3] << 8));
            p->st_pitch_x100 = (int16_t)((uint16_t)p->payload[4] |
                                        ((uint16_t)p->payload[5] << 8));
            p->st_heartbeat  = (uint16_t)((uint16_t)p->payload[6] |
                                         ((uint16_t)p->payload[7] << 8));
            p->st_bad        = (uint16_t)((uint16_t)p->payload[8] |
                                         ((uint16_t)p->payload[9] << 8));
            p->st_timeout    = (uint16_t)((uint16_t)p->payload[10] |
                                         ((uint16_t)p->payload[11] << 8));
            p->st_overflow   = (uint16_t)((uint16_t)p->payload[12] |
                                         ((uint16_t)p->payload[13] << 8));
            p->st_seq++;
        } else {
            p->bad_frame_cnt++;
        }
        break;

    case CARLINK_MSG_SPEED:
        /* 上位机自己发的指令，回环时不消费 */
        break;

    case CARLINK_MSG_CLEAR:
        (void)i;
        break;

    default:
        p->bad_frame_cnt++;
        break;
    }
}

static void cl_frame_end(cl_parser_t *p)
{
    if (p->crc_rx == p->crc_calc) {
        cl_dispatch(p);
        cl_sm_reset(p);
    } else {
        p->bad_frame_cnt++;
        p->retry_flag  = 1u;
        p->retry_start = 1u;
        p->state    = CL_WAIT_H1;
        p->len      = 0u;
        p->msg_id   = 0u;
        p->idx      = 0u;
        p->crc_calc = 0xFFFFu;
        p->crc_rx   = 0u;
    }
}

static void cl_step(cl_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case CL_WAIT_H1:
        if (b == CARLINK_HEAD0) {
            cl_sm_reset(p);
            cl_crc_update(p, b);
            cl_record(p, b);
            p->state = CL_WAIT_H2;
        }
        break;

    case CL_WAIT_H2:
        if (b == CARLINK_HEAD1) {
            cl_crc_update(p, b);
            cl_record(p, b);
            p->state = CL_WAIT_LEN;
        } else if (b == CARLINK_HEAD0) {
            cl_sm_reset(p);
            cl_crc_update(p, b);
            cl_record(p, b);
        } else {
            cl_sm_reset(p);
        }
        break;

    case CL_WAIT_LEN:
        p->len = b;
        if (p->len > CARLINK_MAX_PAYLOAD) {
            p->overflow_cnt++;
            cl_sm_reset(p);
            break;
        }
        cl_crc_update(p, b);
        cl_record(p, b);
        p->idx   = 0u;
        p->state = CL_WAIT_ID;
        break;

    case CL_WAIT_ID:
        p->msg_id = b;
        cl_crc_update(p, b);
        cl_record(p, b);
        p->state = (p->len > 0u) ? CL_WAIT_PAYLOAD : CL_WAIT_CRC_L;
        break;

    case CL_WAIT_PAYLOAD:
        p->payload[p->idx] = b;
        p->idx++;
        cl_crc_update(p, b);
        cl_record(p, b);
        if (p->idx >= p->len) {
            p->state = CL_WAIT_CRC_L;
        }
        break;

    case CL_WAIT_CRC_L:
        p->crc_rx = b;
        cl_record(p, b);
        p->state = CL_WAIT_CRC_H;
        break;

    case CL_WAIT_CRC_H:
        p->crc_rx = (uint16_t)(p->crc_rx | ((uint16_t)b << 8));
        cl_record(p, b);
        cl_frame_end(p);
        break;

    default:
        cl_sm_reset(p);
        break;
    }
}

void carlink_init(cl_parser_t *p, uint32_t now_ms)
{
    memset(p, 0, sizeof(*p));
    p->state        = CL_WAIT_H1;
    p->crc_calc     = 0xFFFFu;
    p->retry_start  = 1u;
    p->last_byte_ms = now_ms;
}

void carlink_feed(cl_parser_t *p, uint8_t b, uint32_t now_ms)
{
    uint8_t guard = 0u;

    p->last_byte_ms = now_ms;
    cl_step(p, b);

    while ((p->retry_flag != 0u) && (guard < 32u)) {
        uint8_t snap[CARLINK_MAX_FRAME];
        uint8_t n     = p->frame_len;
        uint8_t start = p->retry_start;
        uint8_t i;

        guard++;
        if (n > CARLINK_MAX_FRAME) {
            n = CARLINK_MAX_FRAME;
        }
        for (i = 0u; i < n; i++) {
            snap[i] = p->frame_buf[i];
        }

        p->retry_flag  = 0u;
        cl_sm_reset(p);
        p->retry_start = 1u;

        for (i = start; i < n; i++) {
            cl_step(p, snap[i]);
            if (p->retry_flag != 0u) {
                p->retry_start = (uint8_t)(i + 1u);
                break;
            }
        }
        if (p->retry_flag == 0u) {
            break;
        }
    }

    p->retry_flag = 0u;
}

void carlink_check_timeout(cl_parser_t *p, uint32_t now_ms)
{
    if (p->state == CL_WAIT_H1) {
        return;
    }
    if ((uint32_t)(now_ms - p->last_byte_ms) > CARLINK_TIMEOUT_MS) {
        p->timeout_cnt++;
        p->retry_flag = 0u;
        cl_sm_reset(p);
    }
}

uint16_t carlink_crc16(const uint8_t *data, uint16_t n)
{
    uint16_t crc = 0xFFFFu;
    uint8_t  i;

    while (n-- > 0u) {
        crc ^= *data++;
        for (i = 0u; i < 8u; i++) {
            if (crc & 1u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

uint16_t carlink_build_speed(uint8_t *out, int16_t left_rpm, int16_t right_rpm)
{
    uint16_t crc;

    out[0] = CARLINK_HEAD0;
    out[1] = CARLINK_HEAD1;
    out[2] = 4u;
    out[3] = CARLINK_MSG_SPEED;
    out[4] = (uint8_t)((uint16_t)left_rpm & 0xFFu);
    out[5] = (uint8_t)(((uint16_t)left_rpm >> 8) & 0xFFu);
    out[6] = (uint8_t)((uint16_t)right_rpm & 0xFFu);
    out[7] = (uint8_t)(((uint16_t)right_rpm >> 8) & 0xFFu);

    crc = carlink_crc16(out, 8u);
    out[8] = (uint8_t)(crc & 0xFFu);
    out[9] = (uint8_t)(crc >> 8);

    return 10u;
}

uint16_t carlink_build_status(uint8_t *out, int16_t left, int16_t right, int16_t pitch_x100,
                              uint16_t hb, uint16_t bad, uint16_t tmo, uint16_t ovf)
{
    uint16_t crc;

    out[0] = CARLINK_HEAD0;
    out[1] = CARLINK_HEAD1;
    out[2] = 14u;
    out[3] = CARLINK_MSG_STATUS;

    out[4]  = (uint8_t)((uint16_t)left & 0xFFu);
    out[5]  = (uint8_t)(((uint16_t)left >> 8) & 0xFFu);
    out[6]  = (uint8_t)((uint16_t)right & 0xFFu);
    out[7]  = (uint8_t)(((uint16_t)right >> 8) & 0xFFu);
    out[8]  = (uint8_t)((uint16_t)pitch_x100 & 0xFFu);
    out[9]  = (uint8_t)(((uint16_t)pitch_x100 >> 8) & 0xFFu);
    out[10] = (uint8_t)(hb & 0xFFu);
    out[11] = (uint8_t)(hb >> 8);
    out[12] = (uint8_t)(bad & 0xFFu);
    out[13] = (uint8_t)(bad >> 8);
    out[14] = (uint8_t)(tmo & 0xFFu);
    out[15] = (uint8_t)(tmo >> 8);
    out[16] = (uint8_t)(ovf & 0xFFu);
    out[17] = (uint8_t)(ovf >> 8);

    crc = carlink_crc16(out, 18u);
    out[18] = (uint8_t)(crc & 0xFFu);
    out[19] = (uint8_t)(crc >> 8);

    return 20u;
}

uint16_t carlink_build_clear(uint8_t *out)
{
    uint16_t crc;

    out[0] = CARLINK_HEAD0;
    out[1] = CARLINK_HEAD1;
    out[2] = 0u;
    out[3] = CARLINK_MSG_CLEAR;

    crc = carlink_crc16(out, 4u);
    out[4] = (uint8_t)(crc & 0xFFu);
    out[5] = (uint8_t)(crc >> 8);

    return 6u;
}
