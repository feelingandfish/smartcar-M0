# board_notes.md

| 项 | 内容 |
|---|---|
| 主控型号 | STM32F103C8T6，Flash 64KB / SRAM 20KB |
| 晶振 | 外部晶振 8MHz（以丝印为准）；系统主频配到 8MHz（HSI 8MHz，PLL 关闭，AHB/APB1/APB2 均 /1，FLASH_LATENCY_0） |
| 下载口 | SWD；用 ST-LINK/V2，接线 4 根：SWDIO、SWCLK、GND、3.3V |
| LED | PC13（端口 C 第 13 位），低电平亮（LED_ACTIVE_LOW=1） |
| 按键 | PA0（端口 A 第 0 位），按下接地（低电平），用内部上拉（KEY_ACTIVE_LOW=1），软件消抖 20ms |
| 串口 | USART1，TX=PA9、RX=PA10，波特率 115200（8N1） |

怎么查：看丝印 + 万用表量通断 + 查原理图。不许靠猜。如果板子没有原理图，就去量：LED 的正极接哪、限流电阻多大。

---

### 补充计算（考核要求）

**系统主频计算**：SYSCLK = HSI 8MHz（PLL 关闭），AHB = SYSCLK/1 = 8MHz，APB1 = 8MHz，APB2 = 8MHz，FLASH_LATENCY_0。

**波特率分频计算**：USART1 挂 APB2，fCK = 8MHz，USARTDIV = 8,000,000 / (16 × 115200) ≈ 4.34 → BRR = 0x45，实际波特率 ≈ 115,942 bps，误差 <1%。

**消抖位置**：main.c 的 Key_Scan() 内，检测到按键按下后 HAL_Delay(20ms) 再读一次确认，为软件消抖（KEY_DEBOUNCE_MS=20）。

**引脚集中处**：所有引脚/参数集中在 src/board_config.h，现场换板只改这一处。
