# 02. system 层源码讲解

`system/` 是板级公共层。它不负责具体循迹算法，也不负责某一个设备的细节，而是把“这块板子怎么启动、怎么延时、怎么分发中断”统一起来。

本层文件：

```text
system/board.h
system/board.c
system/delay.h
system/delay.c
system/interrupt.h
system/interrupt.c
```

## board.h

`board.h` 是 `board.c` 暴露给外部的接口。你读 `.h` 时重点看两件事：

```text
这个模块给别人用哪些函数？
别人不应该直接碰哪些内部细节？
```

板级模块通常会提供：

```c
void Board_Init(void);
void Board_Task(void);
void Board_ShowBootProgress(...);
```

其中 `Board_Init()` 最重要。`main.c` 应该先调用它，硬件才有可能正常工作。

## board.c：整块板子的初始化总管

`board.c` 包含很多硬件模块头文件：

```c
#include "encoder.h"
#include "gray.h"
#include "jq8400.h"
#include "jy61p.h"
#include "key.h"
#include "link.h"
#include "motor.h"
#include "oled.h"
#include "ti_msp_dl_config.h"
```

这说明 `Board_Init()` 会负责把这些硬件模块都拉起来。

### Board_ShowBootLine()

这是 `static` 函数，只给 `board.c` 内部用。

它做的事情：

1. 准备一个 21 个字符宽的缓冲区。
2. 先填满空格，避免旧字符残留。
3. 把传进来的文本复制进去。
4. 调用 `OLED_ShowLine()` 显示某一行。

为什么要补空格？比如上一行显示过：

```text
UART WAIT
```

后来只显示：

```text
UART OK
```

如果不补空格，屏幕上可能残留成 `UART OKIT`。所以菜单和启动页都会常见“补空格覆盖旧内容”的写法。

### Board_ShowBootProgress()

这个函数把 5 行启动状态写到 OLED：

```text
CLK...
I2C OK
UART WAIT
ADC WAIT
APP WAIT
```

它适合排查上电卡在哪一步。比如屏幕停在 `ADC WAIT`，就说明前面的时钟、I2C、UART 大概率已经过去，后面 ADC 或模块初始化可能有问题。

### Board_Init()

这是 system 层的核心。

当前初始化顺序是分阶段的：

```text
1. SYSCFG_DL_initPower()
2. SYSCFG_DL_GPIO_init()
3. SYSCFG_DL_OLED_init()
4. OLED_Init()
5. 显示启动页
6. SYSCFG_DL_SYSCTL_init()
7. 重新初始化 OLED I2C
8. SYSCFG_DL_PWM_init()
9. 初始化 UART
10. 初始化 ADC0/ADC1
11. 初始化项目硬件模块
12. 设置 OLED 正常显示方向
```

为什么不直接调用 `SYSCFG_DL_init()`？因为拆开后可以让 OLED 先亮，边初始化边显示进度。调试硬件时这比“黑屏卡死”好判断得多。

### 为什么时钟后又调用一次 SYSCFG_DL_OLED_init()

代码里先在系统时钟初始化前配置 OLED，让屏幕尽早能显示；系统时钟切换完成后，又重新配置一次 I2C，让 I2C 工作在正式时钟下。

你可以这样理解：

```text
第一次 OLED init：先能亮，方便看启动进度
第二次 OLED init：时钟稳定后按正式配置工作
```

### 项目模块初始化顺序

```c
Motor_Init();
Gray_Init();
Encoder_Init();
Key_Init();
JY61P_Init();
JQ8400_Init();
Link_Init();
```

这里已经不是纯外设配置，而是项目自己的模块状态初始化：

- 电机先停住并启动 PWM 计数器。
- 灰度加载阈值并采样一次。
- 编码器清零并开 GPIOA 中断。
- 按键开 GPIOB 中断。
- JY61P 打开 UART0 接收中断。
- JQ8400 和 Link 准备串口。

## delay.h / delay.c

`delay.c` 只有一个核心函数：

```c
void delay_ms(uint32_t ms)
{
    delay_cycles((CPUCLK_FREQ / 1000U) * ms);
}
```

这里的 `CPUCLK_FREQ` 来自 `generated/ti_msp_dl_config.h`，当前是 80 MHz。

`delay_cycles()` 是 TI/CMSIS 提供的底层延时函数。它按 CPU 周期空转。

注意这类延时是阻塞式的：

```text
delay_ms(100) 期间，主循环不会继续跑后面的 App_Task 逻辑
```

不过硬件中断通常仍然可以响应。也就是说，按键中断可能发生，但主循环要等 delay 结束后才会处理按键事件。

## interrupt.h / interrupt.c

`interrupt.c` 是中断分发层。它不做复杂逻辑，只把中断交给对应硬件模块。

### GPIOA_IRQHandler()

```c
void GPIOA_IRQHandler(void)
{
    Encoder_HandleGPIOInterrupt();
}
```

GPIOA 上当前主要挂编码器 A/B 相，所以 GPIOA 中断交给编码器模块。

### GPIOB_IRQHandler()

```c
void GPIOB_IRQHandler(void)
{
    Key_HandleGPIOInterrupt();
}
```

GPIOB 上有 PB9/PB8 按键，所以 GPIOB 中断交给按键模块。

### UART0_IRQHandler()

```c
void UART0_IRQHandler(void)
{
    JY61P_HandleUARTInterrupt();
}
```

UART0 是 JY61P，所以 UART0 收到字节后交给 JY61P 模块解析。

## 中断和主循环的关系

中断里尽量少做事。这个工程的风格是：

```text
中断里：记录事件、更新计数、缓存数据
主循环里：读取这些状态，做菜单、控制、电机输出
```

例子：

```text
按键按下
  -> GPIOB 中断
  -> key.c 把 g_keyEvent 改成 KEY_EVENT_1
  -> App_Task 下一轮调用 Key_PopEvent()
  -> app.c 决定菜单确认或返回
```

这样做的好处是中断短，主逻辑清楚。

