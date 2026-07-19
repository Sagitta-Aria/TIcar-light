# m0-light-rtos 接线表

目标芯片：MSPM0G3507。本文与 `config/pin_map.h`、`generated/ti_msp_dl_config.*` 共同描述当前接线。

## 底盘编码电机

| 通道 | 信号 | MCU管脚 | 配置 |
| --- | --- | --- | --- |
| A / 左轮 | PWMA | PA22 | TIMA0 CCP1，20 kHz |
| A / 左轮 | AIN1 | PA31 | GPIO输出 |
| A / 左轮 | AIN2 | PA28 | GPIO输出 |
| A / 左轮 | Encoder A | PB19 | GPIO输入，双边沿中断，内部上拉 |
| A / 左轮 | Encoder B | PB20 | GPIO输入，双边沿中断，内部上拉 |
| B / 右轮 | PWMB | PA12 | TIMA0 CCP3，20 kHz |
| B / 右轮 | BIN1 | PA21 | GPIO输出 |
| B / 右轮 | BIN2 | PA23 | GPIO输出 |
| B / 右轮 | Encoder A | PA13 | GPIO输入，双边沿中断，内部上拉 |
| B / 右轮 | Encoder B | PB24 | GPIO输入，双边沿中断，内部上拉 |

编码器需要与主控共地，并确认输出电平不超过 3.3 V。若实物只有单路编码输出或为开漏输出，不能直接按当前正交解码配置使用。

## 云台步进电机

| 轴 | STEP | DIR | EN |
| --- | --- | --- | --- |
| yaw / 左右 | PA8 | PA9 | 不再由MCU控制，硬件固定有效 |
| pitch / 上下 | PA7 | PB18 | 不再由MCU控制，硬件固定有效 |

PA31 和 PB19 原来分别连接两路云台 EN，现在已经改作 AIN1 和 Encoder1 A。驱动器 EN 必须按实际有效电平固定；当前软件配置沿用“低有效”的假设。

## 核心外设

| 模块 | 信号 | MCU管脚 | 说明 |
| --- | --- | --- | --- |
| OLED | I2C0 SDA / SCL | PA0 / PA1 | 建议 4.7k～10k 外部上拉至 3.3 V |
| K1 / K2 | 按键 | PB9 / PB8 | 低有效，内部上拉 |
| 状态灯 | LED | PA14 | 系统状态指示 |
| H7云台反馈 / 日志TX | UART0 TX / RX | PA10 / PA11 | PA11接H7 UART7_TX/PE8，PA10仅保留日志TX；必须拆开CH340 TX |
| K230视觉 | UART3 TX / RX | PB2 / PB3 | 115200 |
| 板载JY61底座前馈 | UART1 TX / RX | PB6 / PB7 | PB7接JY61 TX；PB6仅保留外设TX功能 |
| HFXT | 晶振 | PA5 / PA6 | 当前软件使用内部32 MHz SYSOSC，硬件位仍保留 |
| SWD | SWDIO / SWCLK | PA19 / PA20 | 禁止复用 |
| BSL invoke | 输入 | PA18 | 不要连接会在启动时拉低的外设 |

## 数字灰度

`CAR_GRAY_INPUT_DIGITAL = 1`，`GRAY_DIGITAL_ACTIVE_HIGH = 1`。

| 灰度 | MCU管脚 | bit |
| --- | --- | --- |
| S1 | PA15 | `0x40` |
| S2 | PA16 | `0x20` |
| S3 | PA17 | `0x10` |
| S4 | PA24 | `0x08` |
| S5 | PA25 | `0x04` |
| S6 | PA26 | `0x02` |
| S7 | PA27 | `0x01` |

## 端口冲突核对

当前底盘十个信号与 OLED、三路 UART、七路灰度、两键、状态灯、云台 STEP/DIR、SWD 和 HFXT 均没有重复 PINCM。特别注意以下旧定义已经失效：

- PA12/PA22 不再是底盘 STEP/DIR，而是 PWMB/PWMA。
- PA13/PB24 不再是底盘 STEP/DIR，而是 Encoder2 A/B。
- PA28 不再是底盘步进 EN，而是 AIN2。
- PA31/PB19 不再是云台 EN，而是 AIN1/Encoder1 A。
- PA21/PA23/PB20 已分配给编码底盘，不再是保留脚。

## 改线同步项

改动任何管脚后必须同步检查：

- `config/pin_map.h`
- `generated/ti_msp_dl_config.h`
- `generated/ti_msp_dl_config.c`
- `hardware/encoder_motor.c`
- `doc/PINOUT.md`
- `README.md`

改完执行：

```powershell
.\tools\build_ccs.ps1 -Clean
```
