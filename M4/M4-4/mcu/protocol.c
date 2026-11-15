/**
  ******************************************************************************
  * protocol.c —— CarLink v1.0 帧解析状态机 + CRC16 + 组帧
  *
  * 设计要点：
  *   1. 逐字节推进状态机，天然支持"半帧跨多次读取"，不假设 read 的边界；
  *   2. LEN 先于载荷被解析，载荷里的 0xAA 0x55 不会被误判为帧头；
  *   3. LEN > 64 直接拒绝，绝不按该长度写入缓冲；
  *   4. 帧内字节间隔 > 50 ms 判超时，丢弃半帧并回找帧头；
  *   5. CRC 失败后从"本帧起始字节 + 1"重新扫描，而不是丢弃整段（不会漏掉重叠的好帧）。
  ******************************************************************************
  */
#include "protocol.h"
#include "string.h"

/* 状态机复位（只清与"当前帧"有关的东西，统计计数保留） */
static void proto_sm_reset(ProtoParser_t *p)
{
    p->state     = PS_WAIT_H1;
    p->len       = 0u;
    p->msg_id    = 0u;
    p->idx       = 0u;
    p->crc_calc  = 0xFFFFu;
    p->crc_rx    = 0u;
    p->frame_len = 0u;
}

/* 边收边算：把 CRC-16/MODBUS 的"单字节推进"拆出来，避免整帧存完再算 */
static void proto_crc_update(ProtoParser_t *p, uint8_t b)
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

/* 记一个原始字节到 frame_buf（供 CRC 失败后重扫） */
static void proto_record(ProtoParser_t *p, uint8_t b)
{
    if (p->frame_len < PROTO_MAX_FRAME) {
        p->frame_buf[p->frame_len] = b;
        p->frame_len++;
    }
}

/* 收到一帧完整字节后的分发 */
static void proto_dispatch(ProtoParser_t *p)
{
    switch (p->msg_id) {
    case MSG_ID_SPEED:
        if (p->len == 4u) {
            p->cmd_left_rpm  = (int16_t)((uint16_t)p->payload[0] |
                                        ((uint16_t)p->payload[1] << 8));
            p->cmd_right_rpm = (int16_t)((uint16_t)p->payload[2] |
                                        ((uint16_t)p->payload[3] << 8));
            if (p->speed_cmd_seq < 255u) {
                p->speed_cmd_seq++;
            } else {
                p->speed_cmd_seq = 1u;
            }
        } else {
            p->bad_frame_cnt++;      /* 长度与 ID 不符 */
        }
        break;

    case MSG_ID_CLEAR:
        if (p->len == 0u) {
            p->clear_req = 1u;
        } else {
            p->bad_frame_cnt++;
        }
        break;

    case MSG_ID_STATUS:
        /* MCU 不消费上位机发的状态帧（仅用于回环自测），静默忽略 */
        break;

    default:
        p->bad_frame_cnt++;          /* 未知消息 ID */
        break;
    }
}

/* 整帧收完：校验 CRC，决定分发还是标记重扫 */
static void proto_frame_end(ProtoParser_t *p)
{
    if (p->crc_rx == p->crc_calc) {
        proto_dispatch(p);
        proto_sm_reset(p);
    } else {
        p->bad_frame_cnt++;
        /* 标记：需要一个"从本帧第 2 个字节开始重扫"的动作，由 Proto_Feed 完成 */
        p->retry_flag  = 1u;
        p->retry_start = 1u;
        /* 只复位状态机字段，frame_buf / frame_len 留给重扫使用 */
        p->state    = PS_WAIT_H1;
        p->len      = 0u;
        p->msg_id   = 0u;
        p->idx      = 0u;
        p->crc_calc = 0xFFFFu;
        p->crc_rx   = 0u;
    }
}

/* 单字节推进状态机（不处理重同步，重同步在 Proto_Feed 里做） */
static void proto_step(ProtoParser_t *p, uint8_t b)
{
    switch (p->state) {
    case PS_WAIT_H1:
        if (b == PROTO_HEAD0) {
            proto_sm_reset(p);
            proto_crc_update(p, b);
            proto_record(p, b);
            p->state = PS_WAIT_H2;
        }
        /* 其它字节原地丢弃，属于"空转"，不计数 */
        break;

    case PS_WAIT_H2:
        if (b == PROTO_HEAD1) {
            proto_crc_update(p, b);
            proto_record(p, b);
            p->state = PS_WAIT_LEN;
        } else if (b == PROTO_HEAD0) {
            /* 连续 AA：把这一字节当作新的帧头第 1 字节，留在本状态 */
            proto_sm_reset(p);
            proto_crc_update(p, b);
            proto_record(p, b);
        } else {
            proto_sm_reset(p);
        }
        break;

    case PS_WAIT_LEN:
        p->len = b;
        if (p->len > PROTO_MAX_PAYLOAD) {
            /* 长度非法：既不分配也不写入，直接回找帧头 */
            p->overflow_cnt++;
            proto_sm_reset(p);
            break;
        }
        proto_crc_update(p, b);
        proto_record(p, b);
        p->idx   = 0u;
        p->state = PS_WAIT_ID;
        break;

    case PS_WAIT_ID:
        p->msg_id = b;
        proto_crc_update(p, b);
        proto_record(p, b);
        p->state = (p->len > 0u) ? PS_WAIT_PAYLOAD : PS_WAIT_CRC_L;
        break;

    case PS_WAIT_PAYLOAD:
        p->payload[p->idx] = b;
        p->idx++;
        proto_crc_update(p, b);
        proto_record(p, b);
        if (p->idx >= p->len) {
            p->state = PS_WAIT_CRC_L;
        }
        break;

    case PS_WAIT_CRC_L:
        p->crc_rx = b;
        proto_record(p, b);
        p->state = PS_WAIT_CRC_H;
        break;

    case PS_WAIT_CRC_H:
        p->crc_rx = (uint16_t)(p->crc_rx | ((uint16_t)b << 8));
        proto_record(p, b);
        proto_frame_end(p);
        break;

    default:
        proto_sm_reset(p);
        break;
    }
}

void Proto_Init(ProtoParser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state       = PS_WAIT_H1;
    p->crc_calc    = 0xFFFFu;
    p->retry_start = 1u;
    p->last_byte_ms = HAL_GetTick();
}

/**
 * 喂一个字节。
 *
 * CRC 失败后的重同步：把刚才那一帧的原始字节（frame_buf）从第 2 个字节起
 * 重新喂给状态机；如果重扫过程中又失败，就从"上次失败的位置 +1"继续滑。
 * 最多滑 32 次后放弃（防止异常数据让主循环卡住），这样保证：
 *   - 不会把本帧内部真正的帧头漏掉；
 *   - 不会无限循环，主循环的响应性不受影响。
 */
void Proto_Feed(ProtoParser_t *p, uint8_t b)
{
    uint8_t guard = 0u;

    p->last_byte_ms = HAL_GetTick();
    proto_step(p, b);

    while ((p->retry_flag != 0u) && (guard < 32u)) {
        uint8_t snap[PROTO_MAX_FRAME];
        uint8_t n     = p->frame_len;
        uint8_t start = p->retry_start;
        uint8_t i;

        guard++;

        if (n > PROTO_MAX_FRAME) {
            n = PROTO_MAX_FRAME;
        }
        for (i = 0u; i < n; i++) {
            snap[i] = p->frame_buf[i];
        }

        p->retry_flag  = 0u;
        proto_sm_reset(p);              /* 清状态机和 frame_len */
        p->retry_start = 1u;

        for (i = start; i < n; i++) {
            proto_step(p, snap[i]);
            if (p->retry_flag != 0u) {
                p->retry_start = (uint8_t)(i + 1u);
                break;
            }
        }
        if (p->retry_flag == 0u) {
            break;                      /* 重扫完成（可能已同步到一帧） */
        }
    }

    p->retry_flag = 0u;                 /* 兜底：绝不把标志留给下一字节 */
}

/* 帧内超时：帧收到一半，超过 50 ms 没新字节 → 丢弃半帧、计入 timeout */
void Proto_CheckTimeout(ProtoParser_t *p, uint32_t now_ms)
{
    if (p->state == PS_WAIT_H1) {
        return;                          /* 没在收帧，不存在超时 */
    }
    if ((uint32_t)(now_ms - p->last_byte_ms) > PROTO_TIMEOUT_MS) {
        p->timeout_cnt++;
        p->retry_flag = 0u;
        proto_sm_reset(p);
    }
}

/* CRC-16/MODBUS：poly 0xA001（反射），init 0xFFFF，输入输出反射，结果异或 0 */
uint16_t Proto_Crc16(const uint8_t *data, uint16_t n)
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

/* 状态回报：LEN=14，MSG=0x02，整帧 20 字节 */
uint16_t Proto_BuildStatus(uint8_t *out,
                           int16_t  left_rpm,
                           int16_t  right_rpm,
                           int16_t  pitch_x100,
                           uint16_t heartbeat,
                           uint16_t bad_frame_cnt,
                           uint16_t timeout_cnt,
                           uint16_t overflow_cnt)
{
    uint16_t crc;

    out[0] = PROTO_HEAD0;
    out[1] = PROTO_HEAD1;
    out[2] = 14u;
    out[3] = MSG_ID_STATUS;

    out[4]  = (uint8_t)((uint16_t)left_rpm & 0xFFu);
    out[5]  = (uint8_t)(((uint16_t)left_rpm >> 8) & 0xFFu);
    out[6]  = (uint8_t)((uint16_t)right_rpm & 0xFFu);
    out[7]  = (uint8_t)(((uint16_t)right_rpm >> 8) & 0xFFu);
    out[8]  = (uint8_t)((uint16_t)pitch_x100 & 0xFFu);
    out[9]  = (uint8_t)(((uint16_t)pitch_x100 >> 8) & 0xFFu);
    out[10] = (uint8_t)(heartbeat & 0xFFu);
    out[11] = (uint8_t)(heartbeat >> 8);
    out[12] = (uint8_t)(bad_frame_cnt & 0xFFu);
    out[13] = (uint8_t)(bad_frame_cnt >> 8);
    out[14] = (uint8_t)(timeout_cnt & 0xFFu);
    out[15] = (uint8_t)(timeout_cnt >> 8);
    out[16] = (uint8_t)(overflow_cnt & 0xFFu);
    out[17] = (uint8_t)(overflow_cnt >> 8);

    crc = Proto_Crc16(out, 18u);
    out[18] = (uint8_t)(crc & 0xFFu);    /* CRC 小端：低字节在前 */
    out[19] = (uint8_t)(crc >> 8);

    return 20u;
}

/* 清计数：LEN=0，MSG=0x03，整帧 6 字节 */
uint16_t Proto_BuildClear(uint8_t *out)
{
    uint16_t crc;

    out[0] = PROTO_HEAD0;
    out[1] = PROTO_HEAD1;
    out[2] = 0u;
    out[3] = MSG_ID_CLEAR;

    crc = Proto_Crc16(out, 4u);
    out[4] = (uint8_t)(crc & 0xFFu);
    out[5] = (uint8_t)(crc >> 8);

    return 6u;
}
