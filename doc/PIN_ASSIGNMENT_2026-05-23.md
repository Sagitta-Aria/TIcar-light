# light-car ccs1.2 接线表

本文按地猛星 MSPM0G3507 最小系统板 H3/H5 排针、板载外设和 ccs1.2 代码配置记录接线。没有写进“当前主控接线”的自由口，先不要随手接外设，避免后续灰度、UART 或下载恢复互相抢脚。

## H3/H5 可见 GPIO

| 排针 | 可见 GPIO |
| --- | --- |
| H3 | PA0, PA1, PA28, PA31, PA2, PB24, PB20, PB19, PB18, PA7, PB2, PB3, PA8, PA9, PB6, PB7 |
| H5 | PA27, PA26, PA25, PA24, PA23, PA22, PA21, PB9, PB8, PA18, PA17, PA16, PA15, PA14, PA13, PA12 |

## 禁止改动/优先保留

| 管脚 | 用途 | 约束 |
| --- | --- | --- |
| PA0 | OLED I2C0 SDA | 开漏释放，必须上拉 |
| PA1 | OLED I2C0 SCL | 开漏释放，必须上拉 |
| PA5/PA6 | HFXT 晶振 | 当前软件不用 PLL，但硬件保留 |
| PA10/PA11 | Type-C CH340 / BSL UART | 正常固件作日志串口，进入 BSL 时复用为下载串口 |
| PA18 | BSL invoke | 保留恢复入口，正常运行时不要外接会拉低的普通外设 |
| PA14 | 状态 LED | 固定作 LED，成功/错误状态从这里看 |
| PA19/PA20 | SWDIO/SWCLK | 下载调试脚，禁止接外设 |
| PA21 | VREF- 相关 | 暂时保留，不做普通 GPIO/ADC |
| PA23 | VREF+ 相关 | 暂时保留，不做普通 GPIO/ADC |
| PB14/PB15/PB16/PB17 | 板载 SPI Flash | 应用层禁止复用 |

## 当前主控接线

| 模块 | 信号 | MCU 管脚 | 说明 |
| --- | --- | --- | --- |
| OLED | SDA | PA0 | I2C0 SDA，外部建议 4.7k~10k 上拉到 3.3V |
| OLED | SCL | PA1 | I2C0 SCL，外部建议 4.7k~10k 上拉到 3.3V |
| 状态灯 | LED | PA14 | 慢闪主循环存活，快闪非致命错误，常亮致命错误 |
| J-Link | SWDIO | PA19 | 只接调试器 |
| J-Link | SWCLK | PA20 | 只接调试器 |
| Key 1 | 输入 | PB9 | 菜单确认 |
| Key 2 | 输入 | PB8 | 菜单切换/返回 |
| Type-C 日志 | UART0 TX/RX | PA10 / PA11 | 115200，接板载 CH340；PA18 拉低进 BSL 时同线复用下载 |
| JY61P | UART1 TX/RX | PB6 / PB7 | 115200，语音模块暂停后释放给姿态模块 |
| Link/反馈总线 | UART3 TX/RX | PB2 / PB3 | 115200，可接视觉模块或后续步进反馈总线 |
| JQ8400 | 暂停接入 | 不接 | 框架保留，不初始化，不占用 UART |

## 四个闭环步进驱动器

电机相线和电机电源接步进驱动器，不接 MCU。MCU 只接驱动器逻辑控制脚，GND 必须共地。

| 电机 | STEP | DIR | EN |
| --- | --- | --- | --- |
| 底盘左电机 | PA7 | PB18 | 不接，驱动器菜单保持使能 |
| 底盘右电机 | PA8 | PA9 | 不接，驱动器菜单保持使能 |
| 云台电机 1 | PA12 | PA22 | 不接，驱动器菜单保持使能 |
| 云台电机 2 | PA13 | PB24 | 不接，驱动器菜单保持使能 |

当前 `hardware/motor.c` 管 DIR 和速度命令，`hardware/stepper_pulse.c` 通过 TIMG0 定时器中断输出 STEP；接线仍是 PA7/PA8/PA12/PA13，不需要因为本次改动重新接线。

## 灰度传感器 ADC

PA14 已固定作 LED，PA18 保留 BSL，PA21/PA23 保留 VREF，因此灰度只接下面 7 路。

| 灰度 | MCU 管脚 | ADC 配置 |
| --- | --- | --- |
| S1 | PA15 | ADC1 CH0 / MEM0 |
| S2 | PA16 | ADC1 CH1 / MEM1 |
| S3 | PA17 | ADC1 CH2 / MEM2 |
| S4 | PA24 | ADC0 CH3 / MEM0 |
| S5 | PA25 | ADC0 CH2 / MEM1 |
| S6 | PA26 | ADC0 CH1 / MEM2 |
| S7 | PA27 | ADC0 CH0 / MEM3 |

## 暂未分配的可用口

| 管脚 | 建议用途 |
| --- | --- |
| PA2 | 普通 GPIO 或后续调试输入 |
| PA28 | 普通 GPIO 备用；UART0 已给 PA10/PA11 日志，不再接 JY61P |
| PA31 | 普通 GPIO 备用；UART0 已给 PA10/PA11 日志，不再接 JY61P |
| PB19 | 普通 GPIO，后续可做步进报警输入 |
| PB20 | 普通 GPIO，后续可做步进到位/报警输入 |

如果后续要恢复 JQ8400，需要重新分配一组真实可用的 UART 引脚；不要直接抢 PB6/PB7，否则会和 JY61P 冲突。
