# m0-light-rtos 接线表

目标芯片：MSPM0G3507。本文与 `config/pin_map.h`、`generated/ti_msp_dl_config.*` 共同描述当前接线。

## 板型开关

构建脚本的`-Board Tianmeng|Dimeng`选择整套引脚；默认是`Tianmeng`：

```powershell
.\tools\build_ccs.ps1 -Profile Gmr -Board Tianmeng -Clean
```

该开关会同时改变`pin_map.h`和生成配置中的相关GPIO/IOMUX，不是只换业务层别名。
下文主表记录当前默认的天猛星接线；两种板型的差异在后文单独列出。

## 底盘编码电机

| 通道 | 信号 | MCU管脚 | 配置 |
| --- | --- | --- | --- |
| B / 左轮 | PWMB | PA12 | TIMA0 CCP3，20 kHz |
| B / 左轮 | BIN1 | PA29 | GPIO输出，避开VREF-网络 |
| B / 左轮 | BIN2 | PA30 | GPIO输出，避开VREF+网络 |
| 左轮 | Encoder A | PA13 | GPIO输入，双边沿中断，内部上拉 |
| 左轮 | Encoder B | PB24 | GPIO输入，双边沿中断，内部上拉 |
| A / 右轮 | PWMA | PA22 | TIMA0 CCP1，20 kHz |
| A / 右轮 | AIN1 | PA31 | GPIO输出 |
| A / 右轮 | AIN2 | PA28 | GPIO输出 |
| 右轮 | Encoder A | PB19 | GPIO输入，双边沿中断，内部上拉 |
| 右轮 | Encoder B | PB20 | GPIO输入，双边沿中断，内部上拉 |

编码器需要与主控共地，并确认输出电平不超过 3.3 V。若实物只有单路编码输出或为开漏输出，不能直接按当前正交解码配置使用。

## Full Profile云台步进电机

| 轴 | STEP | DIR | EN |
| --- | --- | --- | --- |
| yaw / 左右 | PA8 | PA9 | 不再由MCU控制，硬件固定有效 |
| pitch / 上下 | PA7 | PB18 | 不再由MCU控制，硬件固定有效 |

PA31 和 PB19 原来分别连接两路云台 EN，现在已经改作 AIN1 和 Encoder1 A。驱动器 EN 必须按实际有效电平固定；当前软件配置沿用“低有效”的假设。

## 核心外设

| 模块 | 信号 | MCU管脚 | 说明 |
| --- | --- | --- | --- |
| 本地OLED（当前启用） | I2C0 SDA / SCL | PA0 / PA1 | SSD1306菜单显示 |
| K1 / K2 / K3 / K4 / K5 | 按键 | PB0 / PB1 / PB11 / PB10 / PB21 | 低有效，内部上拉；K5已纳入按键驱动 |
| 状态灯 | LED | PB22 | 天猛星板载USER_LED，高电平点亮 |
| Gmr H7 LCD / 任务控制 | UART2 TX | PB15 | 115200 8-N-1；接H7 PE7 RX，PB16不用连接 |
| Full H7云台反馈 / LCD输出 | UART0 TX / RX | PA10 / PA11 | 仅Full使用 |
| 外部M0姿态 / K230视觉 | UART3 TX / RX | PB2 / PB3 | GMR默认接M0姿态，Full接K230；115200 |
| 板载JY61底座前馈 | UART1 TX / RX | PB4 / PB5 | 仅Full默认启用 |
| 地猛星HC-05蓝牙 | UART3 TX / RX | PB2 / PB3 | H3-12/H3-13，115200 8-N-1；启用时替代外部M0姿态 |
| HFXT | 晶振 | PA5 / PA6 | 当前软件使用内部32 MHz SYSOSC，硬件位仍保留 |
| SWD | SWDIO / SWCLK | PA19 / PA20 | 禁止复用 |

当前Gmr固定为天猛星：UART2 PB15单向发送到H7，UART3连接外部M0；UART2 RX、
UART0、UART1、蓝牙和云台STEP资源不初始化。Full仍使用UART0连接H7、UART3
连接K230。模块必须共地。
| BSL invoke | 输入 | PA18 | 不要连接会在启动时拉低的外设 |

## Gmr默认八路红外巡线

Gmr的`CAR_LIBRARY_GRAY_INPUT_METHOD`选择
`CAR_LIBRARY_GRAY_INPUT_INFRARED_GPIO_8`，`GRAY_DIGITAL_ACTIVE_HIGH = 1`。
模块的IR1～IR8从车头朝前按左到右排列，高电平为黑线、低电平为白底。
模块按官方资料使用5 V供电并与主控共地；接入前必须实测各IR输出高电平不超过
3.3 V。MCU内部上拉不能把外部5 V高电平降到安全范围。

| 红外 | MCU管脚 | bit |
| --- | --- | --- |
| IR1 | PA15 | `0x80` |
| IR2 | PA16 | `0x40` |
| IR3 | PA17 | `0x20` |
| IR4 | PA24 | `0x10` |
| IR5 | PA25 | `0x08` |
| IR6 | PA26 | `0x04` |
| IR7 | PA27 | `0x02` |
| IR8 | PB9 | `0x01` |

Full Profile的原七路数字灰度仍保留，不会读取IR8。PB9同时连接天猛星板载Flash
SCK网络，并曾作为IMU660RX SPI1 SCK；Gmr不初始化IMU660RX，且禁止与八路红外
同时启用。

## 端口冲突核对

当前底盘十个信号与四路UART、八路红外、五个按键、状态灯、云台STEP/DIR、SWD和HFXT均没有重复PINCM。本地OLED使用PA0/PA1。特别注意以下旧定义已经失效：

- PA12/PA22 不再是底盘 STEP/DIR，而是 PWMB/PWMA。
- PA13/PB24 不再是底盘 STEP/DIR，而是 Encoder2 A/B。
- PA28 不再是底盘步进 EN，而是 AIN2。
- PA31/PB19 不再是云台 EN，而是 AIN1/Encoder1 A。
- PA21/PA23在天猛星上属于VREF-/VREF+网络，不再用作电机方向脚。
- PB20已分配给右轮Encoder B，不再是保留脚。

## 两种板型切换差异

| 功能 | 地猛星48P | 天猛星64P | 原因 |
| --- | --- | --- | --- |
| 左轮 BIN1 / BIN2 | PA21 / PA23 | PA29 / PA30 | 避开天猛星 VREF- / VREF+ 网络 |
| K1 / K2 | PB9 / PB8 | PB0 / PB1 | 避开板载 SPI Flash 的 SCK / PICO |
| JY61P UART1 TX / RX | PB6 / PB7 | PB4 / PB5 | 避开板载 Flash CS / POCI |
| 板载状态灯 | PA14 | PB22 | 使用天猛星 USER_LED |
| HC-05 UART TX / RX | PB2 / PB3（UART3） | PB15 / PB16（UART2） | 地猛星UART3与外部M0姿态互斥 |

底盘PWM、编码器、云台STEP/DIR、IR1～IR7、OLED、UART0和UART3保持同名脚位。
左右轮按当前TB6612实际通道语义保持，不按旧HTML表的行名再交换一次。

天猛星在`pin_map.h`定义的扩展脚：

| 扩展功能 | 引脚 |
| --- | --- |
| K3 / K4 / K5 | PB21 / PB10 / PB11 |
| 继电器 CTRL | PB27 |
| TIMG12 CCP0 扩展PWM | PA14 |
| 外接 LED1 / LED2 | PB23 / PB25 |
| 板载Flash CS | PB6 |
| IMU SPI1 POCI / PICO / SCK | PB7 / PB8 / PB9 |
| IMU CS / INT1 / INT2 | PB14 / PB17 / PB12 |

除IMU660RX外，这些宏不会主动开启外设。两个Profile的IMU660RX驱动均默认关闭；
选择`CAR_LIBRARY_IMU660RX_AUTO/RA/RB/RC`后才初始化SPI1。PB7-PB9与板载Flash
共享SPI1，驱动访问IMU时会保持Flash CS PB6为高电平。PB14与H8 SPI LCD的CS
是同一引脚，二者不能同时连接使用。完整接线、构建选择和读取接口见
`doc/IMU660RX.md`。

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
