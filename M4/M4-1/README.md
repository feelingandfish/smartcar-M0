[README.md](https://github.com/user-attachments/files/34219245/README.md)
# M4-1 最小系统：点亮板子（README）

从空目录建工程到跑通的完整复现步骤。

本工程使用 **CubeMX + Keil μVision5 MDK + HAL 库**，用 Keil 工程复现考题的构建约定（不用 Makefile，`build/` 目录存放 Keil 编译产物）。

## 1. 硬件清单
- STM32F103C8T6 最小系统板 ×1（板上自带 PC13 LED）
- 面包板 ×1
- ST-LINK/V2（SWD 下载调试）×1
- USB-TTL（CH340）×1
- 按键 ×1、杜邦线若干

## 2. 工具链
- STM32CubeMX（建工程、配置外设）
- Keil μVision5 MDK（编译、烧录）
- ST-Link 驱动（识别 ST-LINK/V2）
- CH340 驱动（识别 USB-TTL 串口）

## 3. 创建工程（CubeMX）
1. 打开 CubeMX → New Project → 选择芯片 **STM32F103C8Tx**
2. 外设配置：
   - **PC13**：GPIO_Output（推挽，初始低电平）→ LED
   - **PA0**：GPIO_Input + 上拉 → 按键
   - **USART1**：Asynchronous 异步，115200 8N1，PA9=TX / PA10=RX
3. Project Manager → 工具链选 **MDK-ARM V5** → 生成代码
4. 生成后工程树中 `Core/Src/main.c`，配置代码会写在 USER CODE 区之间

## 4. 编写功能代码
在 main.c 的 **USER CODE 区**加入功能代码（生成代码不会被覆盖）：

- **Includes 区**：`#include <stdio.h>`、`#include "board_config.h"`
- **USER CODE 0**：
  - `fputc()` —— printf 重定向到 USART1
  - `Key_Scan()` —— 按键边沿检测 + 软件消抖（松开→按下后 delay 20ms 再读确认）
- **USER CODE 1**：`uint32_t last_beat = HAL_GetTick();`
- **while(1)**：
  - 功能A：`Key_Scan()==1` 时翻转 LED + 打印 `[KEY] button pressed`
  - 功能B：`HAL_GetTick()-last_beat >= HEARTBEAT_MS`（500ms）时打印 `[HEART] uptime=... ms`，完全非阻塞

引脚、串口、心跳、消抖参数集中在 `board_config.h`。

完整代码见 `src/main.c` 与 `src/board_config.h`。

## 5. 编译（Keil）
1. 打开 `MDK-ARM/*.uvprojx`
2. Options for Target → Target → 勾选 **Use MicroLIB**（printf 重定向必需）
3. 按 **F7** 编译 → 目标 **0 Error, 0 Warning**

## 6. 烧录（ST-Link）
1. Options → Debugger → 选择 **ST-Link Debugger**
2. Settings → 选 **SW** 模式，能读到 IDCODE（SWDIO）说明芯片已连接
3. 点 **LOAD** → Erase / Programming / Verify OK

> 注意：ST-Link 的 USB 头必须插电脑（只靠板子 USB 供电识别不到）；SWD 的 4 根杜邦线要插牢，松了会报 "No target connected"。

## 7. 功能验证
- **功能A**：按一下按键 → LED 亮；再按一下 → LED 灭（翻转）
- **功能B**：
  1. USB-TTL 接 PA9(RX→PA9)、PA10(TX→PA10)、GND→GND
  2. 串口助手（如 SSCOM）选 COM 口、115200 8N1、打开
  3. 每 500ms 收到一行 `[HEART] uptime=... ms`
  4. 按按键收到 `[KEY] button pressed`
- 实测串口日志见 `docs/serial_log.txt`




```
