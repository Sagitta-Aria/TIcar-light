# 天猛星与地猛星引脚资源报告

日期：2026-07-27  
目标芯片：MSPM0G3507  
工程：`D:\Ti\m0-light-rtos`  
工具链：TI Arm Clang 4.0.4 LTS + FreeRTOS V11.3.0

## 1. 当前双车配置结论

| 项目 | 天猛星主车 | 地猛星从车 |
| --- | --- | --- |
| 构建选择 | `Gmr / Tianmeng / Master` | `Gmr / Dimeng / Slave` |
| HC-05角色 | 主机，地址`00:25:06:01:0D:D3` | 从机，地址`98:DA:50:03:01:77` |
| 蓝牙MCU串口 | UART2，PB15 TX / PB16 RX | UART3，PB2 TX / PB3 RX |
| 蓝牙数据格式 | 115200，8-N-1，无流控 | 115200，8-N-1，无流控 |
| HC-05实机状态 | 尚未在本次操作中修改，仍需AT回读确认 | 已从9600改为115200并回读确认 |
| 外部M0姿态 | UART3 PB2/PB3可用，115200 | 被蓝牙占用，自动关闭 |
| 板载JY61 | UART1外设/中断关闭，引脚复用仍保留 | 当前从机构建默认开启，UART1 PB6/PB7 |
| UART0日志 | UART0外设/中断关闭，引脚复用仍保留 | 当前从机构建默认开启，PA10/PA11 |
| H7 / K230 | Gmr配置关闭 | Gmr配置关闭 |

主车HC-05必须也设置为`AT+UART=115200,0,0`。只修改MCU或只修改其中一个
HC-05都会造成透明数据模式乱码或完全收不到帧。HC-05完整AT模式仍使用固定
38400，不应把`CAR_BLUETOOTH_AT_BAUD_RATE`改成115200。

本次实机AT操作记录：

| 项目 | 回读结果 |
| --- | --- |
| USB转串口 | CH340，COM15 |
| AT探测速率 | 38400，`AT`返回`OK` |
| 模块身份 | `ROLE=0`，`ADDR=98da:50:030177` |
| 修改前 | `+UART=9600,0,0` |
| 执行命令 | `AT+UART=115200,0,0`，返回`OK` |
| 修改后 | `+UART=115200,0,0`，返回`OK` |

## 2. 两板共用的底盘与传感器引脚

| 资源 | 外设/模式 | MCU引脚 | 说明 |
| --- | --- | --- | --- |
| 左轮PWMB | TIMA0 CCP3 | PA12 | 32 MHz / 1600 = 20 kHz PWM |
| 左轮编码器A | GPIO双边沿中断 | PA13 | 内部上拉，正交计数 |
| 左轮编码器B | GPIO双边沿中断 | PB24 | 内部上拉，正交计数 |
| 右轮PWMA | TIMA0 CCP1 | PA22 | 20 kHz PWM |
| 右轮AIN1 | GPIO输出 | PA31 | TB6612方向控制 |
| 右轮AIN2 | GPIO输出 | PA28 | TB6612方向控制 |
| 右轮编码器A | GPIO双边沿中断 | PB19 | 内部上拉，正交计数 |
| 右轮编码器B | GPIO双边沿中断 | PB20 | 内部上拉，正交计数 |
| 灰度S1/S2/S3 | GPIO输入 | PA15 / PA16 / PA17 | 左侧至中左，数字高有效 |
| 灰度S4/S5/S6/S7 | GPIO输入 | PA24 / PA25 / PA26 / PA27 | 中间至最右，数字高有效 |
| OLED SDA/SCL | I2C0，100 kHz | PA0 / PA1 | SSD1306，本地菜单 |
| pitch STEP/DIR | GPIO输出 | PA7 / PB18 | Gmr不驱动云台，Full使用 |
| yaw STEP/DIR | GPIO输出 | PA8 / PA9 | Gmr不驱动云台，Full使用 |
| HFXT IN/OUT | 晶振保留 | PA5 / PA6 | 当前使用内部32 MHz SYSOSC |
| SWDIO/SWCLK | 调试下载 | PA19 / PA20 | 禁止复用或接强驱动外设 |
| BSL invoke | 启动配置输入 | PA18 | 外设不得在复位时拉低 |

云台驱动EN不再由MCU控制。原EN相关的PA31和PB19已经分别用于右轮AIN1和
右轮编码器A，云台EN必须在硬件上按正确有效电平固定。

## 3. 底盘方向、按键、JY61与状态灯差异

| 功能 | 地猛星48P | 天猛星64P | 选择原因 |
| --- | --- | --- | --- |
| 左轮BIN1 / BIN2 | PA21 / PA23 | PA29 / PA30 | 天猛星PA21/PA23属于VREF网络 |
| K1 / K2 | PB9 / PB8 | PB0 / PB1 | 天猛星避开板载SPI Flash |
| JY61 UART1 TX / RX | PB6 / PB7 | PB4 / PB5 | 天猛星PB6/PB7属于Flash/SPI1资源 |
| 状态灯 | PA14 | PB22 | 使用各板板载LED |
| HC-05 TX / RX | PB2 / PB3，UART3 | PB15 / PB16，UART2 | 地猛星没有独立UART2接线方案 |

所有UART表格中的TX/RX均以MCU方向命名：MCU TX必须接模块RXD，MCU RX必须
接模块TXD。UART电平为3.3 V，模块与主控必须共地。

## 4. UART资源与互斥关系

| UART | 固定/候选引脚 | 天猛星Gmr主车 | 地猛星Gmr从车 | 冲突规则 |
| --- | --- | --- | --- | --- |
| UART0 | PA10 TX / PA11 RX | 外设/中断关闭，GPIO复用仍保留 | 默认电脑日志/调参，115200 | PA11不能同时接H7姿态和文本RX |
| UART1 | 天猛星PB4/PB5；地猛星PB6/PB7 | 外设/中断关闭，GPIO复用仍保留 | 默认JY61，115200 | 天猛星避开SPI1 Flash脚 |
| UART2 | PB15 TX / PB16 RX | HC-05主机，115200 | 未分配 | 天猛星蓝牙专用 |
| UART3 | PB2 TX / PB3 RX | 外部M0姿态，115200 | HC-05从机，115200 | 地猛星蓝牙与M0姿态互斥 |

Full Profile把UART3 PB2/PB3交给K230视觉。因此`Full / Dimeng`禁止启用蓝牙；
Gmr Profile不启用K230。天猛星蓝牙使用独立UART2，所以仍可同时保留UART3
外部M0姿态。地猛星蓝牙一旦启用，`CAR_M0_ATTITUDE_UART_REQUIRED`自动为0。

`-Jy61 Disabled`或`-Log Disabled`只跳过UART外设初始化和对应ISR业务。当前
`SYSCFG_DL_GPIO_init()`仍统一设置UART0、UART1和UART3的IOMUX，所以PA10/PA11、
板型对应的JY61引脚以及PB2/PB3仍应视为保留资源，不能直接接成普通GPIO输出。

## 5. 蓝牙具体接线

| 主控板 | MCU信号 | 排针 | 接HC-05 | 电源 |
| --- | --- | --- | --- | --- |
| 天猛星 | PB15 / UART2 TX | U21-11 | RXD | EXT_3V3按模块载板要求供电 |
| 天猛星 | PB16 / UART2 RX | U21-13 | TXD | GND可用U21-21 |
| 地猛星 | PB2 / UART3 TX | H3-12 | RXD | 3.3V可用H3-19 |
| 地猛星 | PB3 / UART3 RX | H3-13 | TXD | GND可用H3-20 |

正常透明传输时KEY保持低电平。进入完整AT模式时按模块载板要求在上电时拉高
KEY，电脑侧使用38400发送AT命令；`AT+UART=115200,0,0`修改的是退出AT模式后
的透明数据速率。

## 6. 天猛星扩展资源

| 扩展功能 | MCU引脚 | 当前状态/限制 |
| --- | --- | --- |
| K3 / K4 / K5 | PB21 / PB10 / PB11 | 仅定义引脚，默认未初始化 |
| 继电器CTRL | PB27 | 默认未初始化 |
| 扩展PWM | PA14 / TIMG12 CCP0 | 默认未初始化；与地猛星状态灯定义不同 |
| 外接LED1 / LED2 | PB23 / PB25 | 默认未初始化 |
| 板载Flash CS | PB6 | SPI1共享总线片选 |
| SPI1 POCI / PICO / SCK | PB7 / PB8 / PB9 | Flash与外接IMU共享 |
| IMU660RX CS | PB14 | 与H8 SPI LCD CS冲突，不能同时连接使用 |
| IMU660RX INT1 / INT2 | PB17 / PB12 | 仅启用IMU库时使用 |

当前Gmr与Full默认都关闭IMU660RX。启用RA/RB/RC/AUTO方案后才初始化SPI1，
访问IMU时软件保持Flash CS PB6为高电平。

## 7. 定时器、ADC与中断资源

| 资源 | 用途 | 当前行为 |
| --- | --- | --- |
| TIMA0 CCP1/CCP3 | 两路底盘PWM | 20 kHz，无周期中断 |
| TIMG0 | 灰度语义快采样 | 10 kHz，仅主车循迹或Task1/Task4需要时启动 |
| TIMG6 | 云台STEP调度 | 20 kHz，仅Full云台流程使用 |
| TIMG12 CCP0 | 天猛星扩展PWM | 只预留，默认不启用 |
| ADC0 / ADC1 | 七路模拟灰度备选 | 当前数字灰度方案不启动ADC转换 |
| GROUP1 GPIO IRQ | 编码器与按键 | 编码器双边沿；按键事件唤醒Input任务 |
| UART IRQ | 通信收发 | 蓝牙优先级2；灰度定时器优先级1 |

## 8. 当前不可随意复用的引脚

- PA0/PA1：OLED I2C0。
- PA5/PA6：板上HFXT网络，即使当前PLL关闭也不建议接普通GPIO负载。
- PA12/PA22：底盘PWM，不再是旧底盘STEP/DIR。
- PA13/PB24/PB19/PB20：四路编码器输入。
- PA15/PA16/PA17/PA24/PA25/PA26/PA27：七路灰度。
- PA18：BSL invoke；启动电平错误会影响正常启动。
- PA19/PA20：SWD下载调试。
- PA28/PA31及板型对应的左轮方向脚：TB6612方向控制。
- 天猛星PA21/PA23：VREF网络，不得作为左轮方向输出。
- 地猛星PB2/PB3：当前从机蓝牙独占，不能再并接M0姿态或K230。
- 天猛星PB15/PB16：当前主机蓝牙独占。

## 9. 配置与验证依据

引脚业务映射以`config/pin_map.h`为准，外设实例与基础IOMUX以
`generated/ti_msp_dl_config.h/.c`为准，资源所有权和编译期冲突检查位于
`config/resource_config.h`。构建时必须显式选择板型和蓝牙角色：

```powershell
.\tools\build_ccs.ps1 -Profile Gmr -Board Tianmeng -BluetoothRole Master -Jy61 Disabled -Log Disabled -Clean
.\tools\build_ccs.ps1 -Profile Gmr -Board Dimeng -BluetoothRole Slave -Clean
```

改线时必须同步核对`doc/PINOUT.md`，不能只改宏名称；SysConfig生成文件不应手工
改动，除非重新生成并完整检查所有板型条件分支。
