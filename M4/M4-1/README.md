# M4-1 最小系统：点亮板子（README）

从空目录建工程到跑通的完整复现步骤。

本工程使用 **CubeMX + Keil μVision5 MDK + HAL 库**，用 Keil 工程复现考题的构建约定（不用 Makefile，`build/` 目录存放 Keil 编译产物）。

---

## 0. 基础概念（先搞清楚这几个词）

### 什么是 HAL 库
HAL = **硬件抽象层**，是 ST 官方写好的"函数库"，把底层寄存器操作封装成简单函数。
- 想让 LED 亮，直接调 `HAL_GPIO_WritePin(...)` 一行搞定，不用自己翻寄存器手册写底层代码
- 里面是 GPIO、串口(UART)、定时器(Timer)、ADC、I2C/SPI 等外设的驱动函数
- CubeMX 生成的代码就是基于 HAL 库的，`main.c` 里所有 `HAL_xxx` 开头的都是

### 什么是 GPIO
GPIO = **通用输入/输出**，就是芯片的那些引脚，每个引脚可配成两种用途：
- **输出模式**：程序控制它输出高/低电平 → 让 LED 亮灭
- **输入模式**：读取引脚现在是高还是低 → 判断按键有没有按下

### 什么是时钟配置（SystemClock_Config）
芯片里所有部件都要靠"时钟节拍"工作。`SystemClock_Config()` 就是设置**系统主频**并给各外设分配时钟的函数。
- HSI = 芯片**自带的内部时钟**（8MHz），不用接外部晶振
- CubeMX 根据你在图形界面选的选项**自动生成**这段配置代码，不用手写寄存器

---

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
  - `fputc()` —— printf 重定向到 USART1（让 printf 能从串口发出来）
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

> F7 就是"一键编译"：自动完成"编译所有源文件 + 链接 + 生成 .hex"，对应 Linux 下的 `make` 命令。

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

### 8.1 系统主频怎么算
公式：`SYSCLK = 时钟源 × PLL倍频 ÷ AHB分频`
本工程：HSI = 8MHz，**PLL 关闭**（不倍频），AHB / APB1 / APB2 均 `/1`
- `SYSCLK = 8MHz × 1 ÷ 1 = 8 MHz`
- `HCLK = SYSCLK/1 = 8MHz`；`APB1 = 8MHz`；`APB2 = 8MHz`

### 8.2 串口波特率分频系数与误差
USART1 挂 APB2，fCK = APB2 = **8MHz**
- `USARTDIV = fCK / (16 × 波特率) = 8,000,000 / (16 × 115200) ≈ 4.34`
- BRR 写入：整数部分 4，小数 0.34×16≈5 → `BRR = 0x45`（实际 USARTDIV = 4.3125）
- 实际波特率 = 8,000,000 / (16 × 4.3125) ≈ **115,942 bps**
- **误差 = (115942 − 115200) / 115200 ≈ 0.64%**（<1%，可正常通信）

### 8.3 按键消抖放在哪一层？为什么？
放在**主循环里**（Key_Scan 函数内），**不用中断**。原因：
1. 按键不是"紧急"事件，人手按压节奏（几十毫秒）远大于主循环周期，主循环检测足够及时
2. 主循环里用 `HAL_Delay(20ms)` 消抖，实现简单、好调试
3. 中断里不宜长时间延时（会阻塞其他中断），且按键优先级低，没必要占用中断资源


