#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
M4-4 协议 v1.0 —— 示例帧生成 / 解析工具

用途：
  1) 生成协议文档中的示例帧（十六进制），保证文档里的字节可被独立复算
  2) 解析任意十六进制字节串，逐字段打印并校验 CRC（考官用它验证你的文档）

依赖：仅 Python 3 标准库

用法：
  python frame_gen.py speed --left 60 --right 60
  python frame_gen.py speed --left -30 --right 45
  python frame_gen.py status --left 60 --right 59 --pitch 12.34 --hb 7 --bad 0 --timeout 0 --overflow 0
  python frame_gen.py clear
  python frame_gen.py crc "AA 55 04 01 3C 00 3C 00"
  python frame_gen.py parse "AA 55 04 01 3C 00 3C 00 EE 1D"
"""

import argparse
import sys

# ---------------- 协议常量（与 docs/protocol_v1.0.md 严格一致） ----------------

HEAD0 = 0xAA          # 帧头第 1 字节
HEAD1 = 0x55          # 帧头第 2 字节
MAX_PAYLOAD = 64      # 载荷长度上限，超过即判非法帧（防止长度字段被干扰后写爆缓冲）

MSG_SPEED = 0x01      # 上位机 -> MCU：速度指令
MSG_STATUS = 0x02     # MCU -> 上位机：状态回报
MSG_CLEAR = 0x03      # 上位机 -> MCU：清零坏帧计数

MSG_NAME = {
    MSG_SPEED: "速度指令 (上位机->MCU)",
    MSG_STATUS: "状态回报 (MCU->上位机)",
    MSG_CLEAR: "清计数 (上位机->MCU)",
}


def crc16_modbus(data: bytes) -> int:
    """CRC-16/MODBUS：多项式 0xA001（0x8005 反射），初值 0xFFFF，输入输出均反射。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(msg_id: int, payload: bytes) -> bytes:
    """组帧：帧头(2) + 长度(1) + ID(1) + 载荷(n) + CRC16(2，小端)"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("载荷长度 %d 超过上限 %d" % (len(payload), MAX_PAYLOAD))
    body = bytes([HEAD0, HEAD1, len(payload), msg_id]) + payload
    crc = crc16_modbus(body)
    # CRC 以小端存放：低字节在前
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def hex_str(data: bytes) -> str:
    return " ".join("%02X" % b for b in data)


def parse_hex(text: str) -> bytes:
    """把 'AA 55 04 01 ...' 或 'AA550401...' 解析成 bytes"""
    cleaned = text.replace(",", " ").replace("0x", " ").replace("0X", " ")
    tokens = cleaned.split()
    out = bytearray()
    for t in tokens:
        if len(t) <= 2:
            out.append(int(t, 16))
        else:  # 允许连写
            for i in range(0, len(t) - len(t) % 2, 2):
                out.append(int(t[i:i + 2], 16))
    return bytes(out)


# ---------------- 载荷编解码 ----------------

def encode_speed(left: int, right: int) -> bytes:
    """速度指令载荷：left_rpm(int16 LE) + right_rpm(int16 LE)，共 4 字节"""
    return bytes([left & 0xFF, (left >> 8) & 0xFF, right & 0xFF, (right >> 8) & 0xFF])


def encode_status(left: int, right: int, pitch_x100: int,
                  heartbeat: int, bad: int, timeout: int, overflow: int) -> bytes:
    """状态回报载荷：7 个 16 位字段，共 14 字节，全部小端"""
    vals = [left, right, pitch_x100, heartbeat, bad, timeout, overflow]
    out = bytearray()
    for v in vals:
        out.append(v & 0xFF)
        out.append((v >> 8) & 0xFF)
    return bytes(out)


def s16(u: int) -> int:
    """无符号 16 位按补码解释为有符号"""
    return u - 0x10000 if u & 0x8000 else u


def decode_payload(msg_id: int, payload: bytes):
    """返回 (字段名字典, 说明行)"""
    fields = {}
    lines = []
    if msg_id == MSG_SPEED:
        if len(payload) != 4:
            raise ValueError("速度指令载荷应为 4 字节，实际 %d" % len(payload))
        l = s16(payload[0] | (payload[1] << 8))
        r = s16(payload[2] | (payload[3] << 8))
        fields = {"left_rpm": l, "right_rpm": r}
        lines.append("  left_rpm  = %d r/min" % l)
        lines.append("  right_rpm = %d r/min" % r)
    elif msg_id == MSG_STATUS:
        if len(payload) != 14:
            raise ValueError("状态回报载荷应为 14 字节，实际 %d" % len(payload))
        u = [payload[i] | (payload[i + 1] << 8) for i in range(0, 14, 2)]
        l, r, pitch = s16(u[0]), s16(u[1]), s16(u[2])
        fields = {
            "left_rpm": l, "right_rpm": r,
            "pitch": pitch / 100.0,
            "heartbeat": u[3], "bad_frame_cnt": u[4],
            "timeout_cnt": u[5], "overflow_cnt": u[6],
        }
        lines.append("  left_rpm      = %d r/min" % l)
        lines.append("  right_rpm     = %d r/min" % r)
        lines.append("  pitch         = %.2f deg (原始 %d × 0.01)" % (pitch / 100.0, pitch))
        lines.append("  heartbeat     = %d" % u[3])
        lines.append("  bad_frame_cnt = %d" % u[4])
        lines.append("  timeout_cnt   = %d" % u[5])
        lines.append("  overflow_cnt  = %d" % u[6])
    elif msg_id == MSG_CLEAR:
        if len(payload) != 0:
            raise ValueError("清计数载荷应为 0 字节，实际 %d" % len(payload))
        fields = {}
        lines.append("  (无载荷：要求 MCU 将三个计数清零)")
    else:
        fields = {"raw": payload.hex()}
        lines.append("  (未知消息 ID，载荷 %d 字节：%s)" % (len(payload), payload.hex().upper()))
    return fields, lines


def do_parse(data: bytes) -> int:
    """按单元格逐字段解析整帧，返回 0 成功 / 1 失败"""
    if len(data) < 6:
        print("[FAIL] 帧长 %d < 最小 6 字节" % len(data))
        return 1
    if data[0] != HEAD0 or data[1] != HEAD1:
        print("[FAIL] 帧头不对：%02X %02X（应为 AA 55）" % (data[0], data[1]))
        return 1
    length = data[2]
    msg_id = data[3]
    if length > MAX_PAYLOAD:
        print("[FAIL] 长度 %d 超过上限 %d" % (length, MAX_PAYLOAD))
        return 1
    total = 4 + length + 2
    if len(data) != total:
        print("[FAIL] 帧长不符：长度字段=%d，应有 %d 字节，实际 %d 字节" % (length, total, len(data)))
        return 1
    payload = data[4:4 + length]
    crc_rx = data[4 + length] | (data[5 + length] << 8)
    crc_calc = crc16_modbus(data[:4 + length])

    print("帧头      : %02X %02X  (OK)" % (data[0], data[1]))
    print("长度      : %d 字节（仅载荷）" % length)
    print("消息 ID   : 0x%02X  %s" % (msg_id, MSG_NAME.get(msg_id, "未知")))
    print("载荷      : %s" % hex_str(payload))
    print("CRC(帧内) : 0x%04X  (小端存放 %02X %02X)" % (crc_rx, data[4 + length], data[5 + length]))
    print("CRC(计算) : 0x%04X" % crc_calc)
    if crc_rx != crc_calc:
        print("[FAIL] CRC 不匹配 —— 该帧应被判为坏帧")
        return 1
    print("[ OK ] CRC 匹配")
    print("字段解析：")
    _, lines = decode_payload(msg_id, payload)
    for ln in lines:
        print(ln)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="M4-4 协议 v1.0 示例帧工具")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("speed", help="生成速度指令帧 (ID=0x01)")
    p.add_argument("--left", type=int, required=True)
    p.add_argument("--right", type=int, required=True)

    p = sub.add_parser("status", help="生成状态回报帧 (ID=0x02)")
    p.add_argument("--left", type=int, default=0)
    p.add_argument("--right", type=int, default=0)
    p.add_argument("--pitch", type=float, default=0.0, help="角度(度)，内部乘 100 存 int16")
    p.add_argument("--hb", type=int, default=0, help="heartbeat 计数")
    p.add_argument("--bad", type=int, default=0, help="bad_frame_cnt")
    p.add_argument("--timeout", type=int, default=0, help="timeout_cnt")
    p.add_argument("--overflow", type=int, default=0, help="overflow_cnt")

    sub.add_parser("clear", help="生成清计数帧 (ID=0x03)")

    p = sub.add_parser("crc", help="只算 CRC（给出不含 CRC 的字节序列）")
    p.add_argument("hex")

    p = sub.add_parser("parse", help="解析一整帧")
    p.add_argument("hex")

    args = ap.parse_args()

    if args.cmd == "speed":
        frame = build_frame(MSG_SPEED, encode_speed(args.left, args.right))
        print(hex_str(frame))
    elif args.cmd == "status":
        frame = build_frame(MSG_STATUS, encode_status(
            args.left, args.right, int(round(args.pitch * 100)),
            args.hb, args.bad, args.timeout, args.overflow))
        print(hex_str(frame))
    elif args.cmd == "clear":
        print(hex_str(build_frame(MSG_CLEAR, b"")))
    elif args.cmd == "crc":
        data = parse_hex(args.hex)
        crc = crc16_modbus(data)
        print("CRC-16/MODBUS = 0x%04X  (小端字节：%02X %02X)" % (crc, crc & 0xFF, (crc >> 8) & 0xFF))
    elif args.cmd == "parse":
        return do_parse(parse_hex(args.hex))
    return 0


if __name__ == "__main__":
    sys.exit(main())
