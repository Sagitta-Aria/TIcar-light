# 01. 启动、板级初始化和中断

## main.c：单片机程序的门口

正常主函数只有三件事：

```c
Board_Init();
App_Init();
while (1) {
    App_Task();
}
```

这就是嵌入式程序最常见的结构：

- 初始化只做一次。
- `while (1)` 永远不退出。
- 所有周期性工作都放在 `App_Task()` 或它调用的模块里。

如果主循环里什么都不调用，程序其实也在运行，只是一直空转，外设不会被完整初始化，OLED 菜单和按键逻辑也不会跑。

## board.c：上电初始化顺序

`Board_Init()` 是硬件启动总入口。当前代码不是一口气调用 `SYSCFG_DL_init()`，而是分阶段初始化，并且用 OLED 显示进度：

1. `SYSCFG_DL_initPower()`：打开外设电源。
2. `SYSCFG_DL_GPIO_init()`：配置引脚功能。
3. `SYSCFG_DL_OLED_init()` 和 `OLED_Init()`：先把 OLED 拉起来。
4. `SYSCFG_DL_SYSCTL_init()`：配置系统时钟。
5. `SYSCFG_DL_PWM_init()`：配置电机 PWM 定时器。
6. `SYSCFG_DL_JY61P_init()`、`SYSCFG_DL_JQ8400_init()`、`SYSCFG_DL_Exchange_init()`：配置几个 UART。
7. `SYSCFG_DL_GRAY_ADC0_init()`、`SYSCFG_DL_GRAY_ADC1_init()`：配置两个 ADC 外设。
8. `Motor_Init()`、`Gray_Init()`、`Encoder_Init()`、`Key_Init()`、`JY61P_Init()`、`JQ8400_Init()`、`Link_Init()`：初始化项目自己的模块。

这里有一个很实用的调试思路：OLED 先亮，再一步步显示 `CLK OK`、`UART OK`、`ADC OK`。如果卡住，就能大概知道卡在哪个初始化阶段。

## generated/ti_msp_dl_config.c：底层外设配置

`generated/` 下面的文件一般来自 TI SysConfig 或类似配置工具，主要负责把芯片寄存器配置好。

你现在只需要知道它做了这些事：

- CPU 主频配置到 `CPUCLK_FREQ = 80000000`，也就是 80 MHz。
- OLED 用 `I2C0`，PA0 是 SDA，PA1 是 SCL，速率 100 kHz。
- JY61P 用 `UART0`，PA28 TX，PA31 RX，115200。
- JQ8400 用 `UART1`，PB6 TX，PB7 RX，115200。
- Exchange/Link 用 `UART3`，PB2 TX，PB3 RX，115200。
- 灰度传感器用了 `ADC0` 和 `ADC1` 两个 ADC 外设。
- 电机 PWM 用 `TIMA0` 的两个比较通道。
- 按键 PB9/PB8、编码器 PA12/PA13/PA22/PA23 配成 GPIO 输入和中断。

这类生成文件通常不适合初学阶段从头细看，因为里面大量是 TI DriverLib 调寄存器配置。先从 `pin_map.h` 和各模块的 `.c` 文件理解更舒服。

## pin_map.h：给生成名字起项目别名

`pin_map.h` 的作用是把生成文件里的名字转换成项目里更好读的名字。

比如：

```c
#define PIN_KEY_1 KEY_1_PIN
#define PIN_KEY_2 KEY_2_PIN
```

读代码时看到 `PIN_KEY_1`，你马上知道这是项目里的 1 号按键，而不用每次记 `KEY_1_PIN` 具体来自哪里。

灰度这里：

```c
#define PIN_GRAY_ADC0 GRAY_ADC0_INST
#define PIN_GRAY_ADC1 GRAY_ADC1_INST
```

意思是项目里叫 `PIN_GRAY_ADC0`，实际底层就是生成文件里的 `GRAY_ADC0_INST`，也就是芯片外设 `ADC0`。

## board_config.h：项目调参表

`board_config.h` 不是函数逻辑，而是很多参数：

- `CAR_TRACK_BASE_DUTY`：循迹基础速度。
- `CAR_TRACK_TURN_GAIN`：灰度误差换成转向修正的比例。
- `CAR_ENABLE_SPEED_CONTROL`：是否启用编码器速度闭环。
- `CAR_SPEED_KP`、`CAR_SPEED_KI`：速度 PI 参数。
- `GRAY_SENSOR_COUNT`：灰度传感器数量，当前是 7。
- `GRAY_FILTER_SAMPLE_COUNT`：每次灰度更新采几次再平均。
- `GRAY_DEFAULT_THRESHOLD`：灰度默认阈值。

你调车时很多时候不是改算法，而是先改这些参数。

## interrupt.c：中断分发

`interrupt.c` 很短，但很关键：

```c
void GPIOA_IRQHandler(void)
{
    Encoder_HandleGPIOInterrupt();
}

void GPIOB_IRQHandler(void)
{
    Key_HandleGPIOInterrupt();
}

void UART0_IRQHandler(void)
{
    JY61P_HandleUARTInterrupt();
}
```

人话版：

- GPIOA 中断来了，说明编码器引脚可能跳变，交给编码器模块。
- GPIOB 中断来了，说明按键 PB8/PB9 可能触发，交给按键模块。
- UART0 收到数据，说明 JY61P 发来了字节，交给 JY61P 解析模块。

主循环和中断是两条执行路径：

```mermaid
flowchart LR
    A["主循环 App_Task()"] --> B["正常周期任务"]
    C["硬件事件"] --> D["中断函数"]
    D --> E["更新事件/计数/缓存"]
    E --> A
```

所以像按键这种事情，不是在主循环里一直死等按键，而是按键中断先记录一个事件，主循环下一轮再取走事件处理。

## delay.c：延时

`delay_ms(ms)` 用 CPU 周期数实现毫秒延时：

```c
delay_cycles((CPUCLK_FREQ / 1000U) * ms);
```

`CPUCLK_FREQ` 是 80000000，除以 1000 后就是 1 毫秒大约需要的 CPU 周期数。

这种延时是阻塞式的：延时期间 CPU 基本就在等。不过中断通常仍然可以打断它。

