#!/usr/bin/env bash
# ---------------------------------------------------------------
# 破坏测试 2：运行中往串口插入脏字节 / 半截帧
# 目的：验证 MCU 不死机、坏帧计数递增、状态机能从任意错位重同步
#
# 用法（在 linux 目录下）：
#   sudo ./inject_test.sh
#   sudo ./inject_test.sh /dev/ttyUSB0 14
#
# 观察点（跑完看 carlink 打印的统计）：
#   MCU 报告 bad  : 应比测试开始时大（纯脏字节被丢弃 / 错位帧头被计入）
#   MCU 报告 tmo  : 应比测试开始时大（半截帧触发帧内超时）
#   收到状态帧    : 应仍然接近「时长 × 20」，说明链路没被打断
# ----------------------------------------------------------
set -u

PORT="${1:-/dev/ttyUSB0}"
DUR="${2:-14}"

if [ ! -e "$PORT" ]; then
  echo "错误：$PORT 不存在。先在 PowerShell 里执行 usbipd attach --wsl --busid 2-3"
  exit 1
fi

echo "=================================================================="
echo " 端口: $PORT    时长: ${DUR}s    目标: 60 r/min"
echo " 测试期间不要拔线，脚本会自动在指定时刻插入脏数据"
echo "=================================================================="

# 提前验证 sudo 凭证，避免脚本跑到一半卡在密码输入
sudo -v || { echo "sudo 验证失败"; exit 1; }

sudo ./carlink --port "$PORT" --speed 60 --duration "$DUR" &
CPID=$!

sleep 3
echo ""
echo ">>> [第 3 秒] 插入 4 个纯脏字节: 00 FF 12 34"
sudo bash -c "printf '\x00\xFF\x12\x34' > $PORT"
sleep 2

echo ">>> [第 5 秒] 插入半截帧: AA 55 04 01 3C 00 （有帧头无 CRC，应触发帧内超时 tmo+1）"
sudo bash -c "printf '\xAA\x55\x04\x01\x3C\x00' > $PORT"
sleep 2

echo ">>> [第 7 秒] 插入错位帧头: AA AA 55 00 55 （假的 AA 55，应被跳过并重新同步）"
sudo bash -c "printf '\xAA\xAA\x55\x00\x55' > $PORT"
sleep 2

echo ">>> [第 9 秒] 插入被篡改的合法帧: AA 55 04 01 3C 00 3C 00 FF FF （CRC 错，应计 bad+1）"
sudo bash -c "printf '\xAA\x55\x04\x01\x3C\x00\x3C\x00\xFF\xFF' > $PORT"

wait $CPID 2>/dev/null

echo ""
echo "=================================================================="
echo " 测试结束。检查上面统计："
echo "   bad 应该 >= 1（被篡改的那帧）"
echo "   tmo 应该 >= 1（半截帧超时）"
echo "   收到状态帧 应该仍然接近 $((DUR * 20))"
echo "=================================================================="
