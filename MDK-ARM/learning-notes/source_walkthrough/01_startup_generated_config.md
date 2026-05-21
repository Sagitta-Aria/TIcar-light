# 01. 启动汇编、generated 和 config

这一章讲的是“代码能跑起来的地基”。这部分看起来离业务远，但它决定了外设名字、引脚、时钟、中断入口。

## MDK-ARM/startup_mspm0g350x_uvision.s

这个文件是 Keil 工程的启动文件，属于 TI/CMSIS 风格的汇编启动代码。

你先抓这几块：

### 栈和堆

```asm
Stack_Size EQU 0x00000100
Heap_Size  EQU 0x00000000
```

栈是函数调用、局部变量、中断现场保存会用到的内存区域。这里栈大小是 `0x100`，也就是 256 字节。

堆是 `malloc` 这类动态内存会用的区域。这里堆大小是 0，说明工程基本不打算用动态内存。单片机项目常常避免 `malloc`，因为内存小，也不想引入碎片和不可控风险。

### 中断向量表

```asm
__Vectors DCD __initial_sp
          DCD Reset_Handler
          DCD NMI_Handler
          DCD HardFault_Handler
          ...
          DCD UART0_IRQHandler
          DCD TIMA0_IRQHandler
          DCD I2C0_IRQHandler
```

向量表就是“硬件事件到函数地址的表”。芯片复位时会找 `Reset_Handler`，UART0 中断来了会找 `UART0_IRQHandler`。

这解释了为什么你在 `system/interrupt.c` 里写：

```c
void UART0_IRQHandler(void)
```

硬件就能找到它：启动文件里向量表有这个名字。

### Reset_Handler

```asm
Reset_Handler
    IMPORT __main
    LDR R0, =__main
    BX R0
```

复位后并不是直接跳到你写的 `main()`，而是先进入 C 运行库的 `__main`。`__main` 会做一些 C 语言运行环境准备，比如初始化全局变量，然后再调用 `main()`。

### 默认中断

很多中断默认都指向一个死循环：

```asm
Default_Handler
    B .
```

`B .` 的意思是跳到自己，也就是卡住。如果某个中断触发了，但你没有实现对应的 `xxx_IRQHandler`，程序可能就停在默认中断里。

这也是调试时要知道的：莫名卡死，有时是进了没写处理函数的中断。

## generated/ti_msp_dl_config.h

这个头文件主要放宏定义和初始化函数声明。

### 芯片和 DriverLib

```c
#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507
#include <ti/driverlib/driverlib.h>
```

这告诉工程当前芯片系列和具体型号，并引入 TI DriverLib。

### 主频

```c
#define CPUCLK_FREQ 80000000
```

主频 80 MHz。`delay_ms()` 会用这个值计算延时周期。

### PWM

```c
#define PWM_INST TIMA0
#define PWM_PERIOD_COUNTS (4000U)
#define GPIO_PWM_C0_PIN DL_GPIO_PIN_8
#define GPIO_PWM_C1_PIN DL_GPIO_PIN_7
```

含义：

- 电机 PWM 用定时器 `TIMA0`。
- PWM 周期计数是 4000。
- 两个 PWM 输出脚是 PA8 和 PA7。

上层 `motor.c` 不直接写 `TIMA0`，而是通过 `pin_map.h` 里的别名访问。

### OLED I2C

```c
#define OLED_INST I2C0
#define OLED_BUS_SPEED_HZ 100000
#define GPIO_OLED_SDA_PIN DL_GPIO_PIN_0
#define GPIO_OLED_SCL_PIN DL_GPIO_PIN_1
```

OLED 用 I2C0，SDA 是 PA0，SCL 是 PA1，I2C 速率 100 kHz。

### UART 模块

```text
JY61P    UART0  PA28 TX / PA31 RX
JQ8400   UART1  PB6 TX  / PB7 RX
Exchange UART3  PB2 TX  / PB3 RX
```

三个串口都是 115200。只是 Exchange 的 UART3 工作时钟是 80 MHz，JY61P/JQ8400 是 40 MHz，所以波特率分频值不同。

### 灰度 ADC

```c
#define GRAY_ADC0_INST ADC0
#define GRAY_ADC1_INST ADC1
```

这里是两个 ADC 外设实例，不是两个传感器。

灰度 7 路结果槽：

```text
S1 -> ADC0 MEM0
S2 -> ADC1 MEM0
S3 -> ADC1 MEM1
S4 -> ADC1 MEM2
S5 -> ADC0 MEM3
S6 -> ADC0 MEM1
S7 -> ADC0 MEM2
```

注意 `GRAY_ADC0_MEM_GRAY6` 里的 `GRAY6` 是传感器编号，不代表硬件 MEM6。

### 按键和编码器

```text
KEY1 PB9
KEY2 PB8
左编码器 A/B PA12/PA13
右编码器 A/B PA22/PA23
```

这些宏会被 `pin_map.h` 包装成更统一的项目名字。

## generated/ti_msp_dl_config.c

这个 `.c` 文件是真正调用 TI DriverLib 配外设的地方。

### SYSCFG_DL_init()

```c
SYSCFG_DL_initPower();
SYSCFG_DL_GPIO_init();
SYSCFG_DL_SYSCTL_init();
SYSCFG_DL_PWM_init();
SYSCFG_DL_OLED_init();
...
```

这是“一键初始化全部外设”的函数。但当前 `Board_Init()` 没有直接调用它，而是拆开分阶段调用，方便 OLED 显示启动进度。

### SYSCFG_DL_initPower()

做两类事：

- reset：复位外设模块。
- enablePower：打开外设电源。

单片机外设通常不是上电就能用，需要先打开电源、复位、配置。

### SYSCFG_DL_GPIO_init()

它把每个引脚配置成对应功能：

- PWM 引脚配置成定时器输出。
- OLED 引脚配置成 I2C。
- UART 引脚配置成 TX/RX。
- 灰度引脚配置成 ADC 模拟输入。
- 电机方向脚配置成 GPIO 输出。
- 按键和编码器配置成 GPIO 输入和中断。

这一步很重要：如果引脚复用没配对，即使上层代码写对，也不会有正确电平或外设功能。

### SYSCFG_DL_SYSCTL_init()

配置系统时钟，包括：

- flash 等待周期。
- 系统振荡器。
- HFXT / SYSPLL。
- MCLK 时钟来源。

这部分不建议初学时逐行改。时钟错了，UART 波特率、I2C、PWM、delay 都可能跟着错。

### SYSCFG_DL_PWM_init()

配置 `TIMA0` 为 PWM 模式，两个比较通道初始值为 0。

后面 `motor.c` 会调用：

```c
DL_Timer_setCaptureCompareValue(...)
```

改变比较值，也就是改变 PWM 占空比。

### SYSCFG_DL_OLED_init()

配置 I2C0 控制器，包括时钟、滤波、FIFO、clock stretching，然后启用控制器。

OLED 的屏幕命令本身不在 generated 里，而是在 `hardware/oled.c` 里。

### SYSCFG_DL_JY61P_init() / JQ8400 / Exchange

这三个函数配置 UART：

- 时钟。
- 普通 UART 模式。
- TX/RX 双向。
- 8 数据位、无校验、1 停止位。
- 波特率 115200。
- 启用 UART。

协议解析不在这里，JY61P 的解析在 `hardware/jy61p.c`。

### SYSCFG_DL_GRAY_ADC0_init() / GRAY_ADC1_init()

配置 ADC 序列采样。

ADC0 配了三次 conversion memory：

```text
GRAY1 -> ADC input channel 12 -> MEM0
GRAY6 -> ADC input channel 3  -> MEM1
GRAY7 -> ADC input channel 2  -> MEM2
```

ADC1 配了四次：

```text
GRAY2 -> channel 0 -> MEM0
GRAY3 -> channel 1 -> MEM1
GRAY4 -> channel 2 -> MEM2
GRAY5 -> channel 3 -> MEM3
```

`gray.c` 里 `g_grayMap` 正是按照这个配置去读结果。

## config/pin_map.h

这个文件很短，但非常关键。它把 generated 里的长名字改成模块更好读的名字。

例如：

```c
#define PIN_MOTOR_PWM_TIMER PWM_INST
#define PIN_KEY_1 KEY_1_PIN
#define PIN_GRAY_ADC0 GRAY_ADC0_INST
```

这样 `hardware/motor.c`、`hardware/key.c`、`hardware/gray.c` 不需要关心底层命名风格，统一使用 `PIN_` 开头的项目名字。

## config/board_config.h

这是全工程的参数中心。

### 电机和循迹参数

```c
CAR_MOTOR_PWM_MAX_COUNTS
CAR_TRACK_BASE_DUTY
CAR_TRACK_TURN_GAIN
```

这些决定 PWM 上限、基础速度、转向修正比例。

### 丢线和异常参数

```c
CAR_TRACK_LOST_HOLD_TICKS
CAR_TRACK_LOST_SEARCH_TICKS
CAR_TRACK_ADC_FAULT_STOP_TICKS
```

这些决定丢线后先保持多久、搜线多久、ADC 连续失败几次停车。

### App 调度参数

```c
CAR_APP_LOOP_DELAY_MS
CAR_MENU_REFRESH_MS
CAR_MENU_LINK_PRINT_MS
```

主循环大约每 10ms 一轮，菜单和串口监视不是每轮都刷新，而是按 tick 换算。

### 速度闭环参数

```c
CAR_ENABLE_SPEED_CONTROL
CAR_SPEED_KP
CAR_SPEED_KI
CAR_SPEED_MIN_ACTIVE_DUTY
```

`CAR_ENABLE_SPEED_CONTROL` 为 1 时，上层 `Motion_SetSpeed()` 会走编码器闭环；为 0 时会直接开环控制电机 PWM。

### 路线参数

```c
CAR_ROUTE_EDGE_TICKS
CAR_ROUTE_APPROACH_TICKS
CAR_ROUTE_TURN_TOLERANCE_DEG
```

路线层用编码器 tick 判断接近拐角，用 JY61P yaw 判断是否转够 90 度。

### 灰度参数

```c
GRAY_SENSOR_COUNT
GRAY_ACTIVE_HIGH
GRAY_ADC_MAX_VALUE
GRAY_DEFAULT_THRESHOLD
GRAY_FILTER_SAMPLE_COUNT
```

灰度传感器 7 路，ADC 最大 4095，默认阈值 2000，每次更新采 5 次求平均。
