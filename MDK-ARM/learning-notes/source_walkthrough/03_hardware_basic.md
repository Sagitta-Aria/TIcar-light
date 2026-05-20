# 03. hardware 基础模块：电机、按键、编码器、Link

这一章先看几个相对短的硬件模块。它们是后面循迹、速度闭环、菜单交互的基础。

本章文件：

```text
hardware/motor.h
hardware/motor.c
hardware/key.h
hardware/key.c
hardware/encoder.h
hardware/encoder.c
hardware/link.h
hardware/link.c
```

## motor.h：电机模块对外接口

`motor.h` 先定义两个枚举。

### MotorId

```c
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} MotorId;
```

这表示你要控制左电机还是右电机。

### MotorDir

```c
typedef enum {
    MOTOR_COAST = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_BRAKE
} MotorDir;
```

这表示电机驱动方式：

- `MOTOR_FORWARD`：正转。
- `MOTOR_REVERSE`：反转。
- `MOTOR_BRAKE`：刹车。
- `MOTOR_COAST`：滑行，两个方向脚都关。

对外函数：

```c
Motor_Init()
Motor_Set()
Motor_SetSpeed()
Motor_Stop()
```

## motor.c：TB6612 电机控制

电机模块控制两个东西：

```text
PWM：速度大小
方向 GPIO：正转、反转、刹车、滑行
```

### Motor_ClampDuty()

```c
return (duty > CAR_MOTOR_PWM_MAX_COUNTS) ? CAR_MOTOR_PWM_MAX_COUNTS : duty;
```

这是限幅。防止上层传入超过 PWM 周期的值。

当前最大值来自：

```c
#define CAR_MOTOR_PWM_MAX_COUNTS (4000U)
```

### Motor_DutyFromSigned()

`Motor_SetSpeed()` 接收有符号速度，例如：

```text
900   正转，占空比 900
-900  反转，占空比 900
```

`Motor_DutyFromSigned()` 就是把负号去掉，得到 PWM 大小。

### Motor_SetDirPins()

这个函数根据 `MotorDir` 设置 TB6612 的两个方向脚。

可以记成表：

```text
FORWARD  IN1=1 IN2=0
REVERSE  IN1=0 IN2=1
BRAKE    IN1=1 IN2=1
COAST    IN1=0 IN2=0
```

它用的是 TI DriverLib：

```c
DL_GPIO_setPins(...)
DL_GPIO_clearPins(...)
```

### Motor_Init()

```c
Motor_Stop();
DL_Timer_startCounter(PIN_MOTOR_PWM_TIMER);
```

初始化时先停车，再启动 PWM 定时器计数。

这很重要：上电初始化过程中电机不应该突然转。

### Motor_Set()

这是最底层的“指定某个电机怎么转”：

```c
Motor_Set(MOTOR_LEFT, MOTOR_FORWARD, 900);
```

会设置左电机方向脚，并把对应 PWM 比较值设成 900。

### Motor_SetSpeed()

这是上层最常用的电机接口：

```c
Motor_SetSpeed(left, right);
```

它通过正负号判断方向：

- `left >= 0`：左轮正转。
- `left < 0`：左轮反转。

上层循迹、速度闭环最终都会变成左右轮的有符号命令。

## key.h：按键类型

`key.h` 有两个枚举：

```c
typedef enum {
    KEY_ID_1 = 0,
    KEY_ID_2
} KeyId;
```

这是“读哪个实体按键”。

```c
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_1,
    KEY_EVENT_2
} KeyEvent;
```

这是“发生了哪个按键事件”。

两者区别：

```text
KeyId    用于主动读取某个键现在是不是按下
KeyEvent 用于主循环处理刚刚发生的按键事件
```

## key.c：按键中断到事件

关键变量：

```c
static volatile KeyEvent g_keyEvent = KEY_EVENT_NONE;
```

解释：

- `static`：只在 `key.c` 内部可见。
- `volatile`：中断里会改它，编译器每次都要重新读。

### Key_PinFromId()

把 `KEY_ID_1` / `KEY_ID_2` 转成真实 pin：

```text
KEY_ID_1 -> PIN_KEY_1 -> PB9
KEY_ID_2 -> PIN_KEY_2 -> PB8
```

### Key_Init()

```c
NVIC_EnableIRQ(GPIOB_INT_IRQn);
```

打开 GPIOB 中断。PB8/PB9 按键中断才能进 `GPIOB_IRQHandler()`。

### Key_IsPressed()

这个函数直接读 GPIO 当前电平。

```c
return (DL_GPIO_readPins(PIN_KEY_PORT, pin) & pin) ? 1U : 0U;
```

如果对应 pin 那一位是 1，就返回 1。

注意：这里是否表示“按下”，取决于硬件和 generated 里的上下拉、触发配置。读代码时先记住它按当前工程约定返回 1 为按下。

### Key_HandleGPIOInterrupt()

这个函数在 GPIOB 中断里被调用。

它用：

```c
DL_GPIO_getPendingInterrupt(PIN_KEY_PORT)
```

判断是哪一路 GPIO 触发了中断，然后设置：

```c
g_keyEvent = KEY_EVENT_1;
```

或：

```c
g_keyEvent = KEY_EVENT_2;
```

### Key_PopEvent()

```c
KeyEvent event = g_keyEvent;
g_keyEvent = KEY_EVENT_NONE;
return event;
```

`Pop` 的含义是“取走”。主循环拿到事件后，内部缓存就清空，避免一个按键事件被重复处理很多次。

## encoder.h / encoder.c：编码器计数

编码器模块保存两个累计计数：

```c
static volatile int32_t g_leftCount;
static volatile int32_t g_rightCount;
```

编码器 A/B 相跳变时，GPIOA 中断会进入：

```c
Encoder_HandleGPIOInterrupt();
```

### Encoder_Init()

```c
Encoder_Reset();
NVIC_EnableIRQ(GPIOA_INT_IRQn);
```

先清零，再打开 GPIOA 中断。

### Encoder_GetLeft() / Encoder_GetRight()

这两个函数只是返回累计计数。

速度闭环不会直接关心每个边沿，只关心一段时间内计数变化：

```text
actualTicks = currentCount - lastCount
```

### Encoder_HandleGPIOInterrupt()

这个函数读当前 A/B 相状态：

```c
state = DL_GPIO_readPins(PIN_ENCODER_PORT, A|B|A|B);
```

然后根据 pending interrupt 判断是哪一路跳变，再看另一相信号决定加 1 还是减 1。

这就是常见的 AB 相编码器方向判断。

你现在不用背每个 case 的正负，只要知道：

```text
A/B 相谁先谁后，决定轮子方向
代码用这个关系更新 g_leftCount/g_rightCount
```

如果实车发现前进时编码器计数是负的，不一定要改编码器底层，也可以通过 `board_config.h` 里的 `CAR_SPEED_LEFT_ENCODER_SIGN` / `CAR_SPEED_RIGHT_ENCODER_SIGN` 修正。

## link.h / link.c：调试串口输出

Link 使用的是 Exchange UART3。

对外函数：

```c
Link_SendByte()
Link_SendBytes()
Link_SendString()
Link_Task()
```

### Link_Init()

```c
NVIC_EnableIRQ(Exchange_INST_INT_IRQN);
```

打开 UART3 中断。不过当前 `interrupt.c` 里还没有 `UART3_IRQHandler()` 的实际分发，所以现在主要还是发送调试字符串。

### Link_SendByte()

```c
DL_UART_transmitDataBlocking(Exchange_INST, data);
```

阻塞发送 1 个字节。阻塞的意思是发送没完成前函数不会返回。

### Link_SendBytes()

检查指针不是空，再循环发每个字节。

```c
if (data == 0) {
    return;
}
```

这是防止传进空指针导致程序访问非法地址。

### Link_SendString()

发送 C 字符串，直到遇到 `'\0'` 结束。

状态机里经常用：

```c
Link_SendString("state: tracking\r\n");
```

`\r\n` 是串口终端常用换行。

