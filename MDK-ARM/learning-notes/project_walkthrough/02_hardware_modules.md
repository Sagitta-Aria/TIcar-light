# 02. 硬件模块逐个看

硬件层的目标是：把底层 `DL_GPIO_xxx`、`DL_ADC12_xxx`、`DL_UART_xxx` 这些 DriverLib 调用，包装成更像“人的动作”的函数，比如 `Motor_SetSpeed()`、`Gray_Update()`、`OLED_ShowLine()`。

## motor.c：电机和 TB6612

电机模块控制两个东西：

- 方向脚：TB6612 的 AIN1/AIN2、BIN1/BIN2。
- PWM 占空比：决定电机速度大小。

核心接口：

```c
void Motor_Set(MotorId motor, MotorDir dir, uint16_t duty);
void Motor_SetSpeed(int16_t left, int16_t right);
void Motor_Stop(void);
```

`Motor_Set()` 是底层接口：指定左/右电机、正转/反转/刹车/滑行、占空比。

`Motor_SetSpeed()` 更好用：传入有符号数。

- 正数：正转。
- 负数：反转。
- 绝对值：PWM 大小。

比如：

```c
Motor_SetSpeed(900, 700);
```

表示左轮速度命令 900，右轮 700，小车会往右侧修正或转弯。

`Motor_Stop()` 当前用 `MOTOR_COAST`，也就是两个方向脚都清零，电机滑行停，不是强刹车。

## key.c：PB9/PB8 按键

按键有两层概念：

- `KeyId`：你想读哪个按键。
- `KeyEvent`：中断记录下来的“发生了哪个按键事件”。

当前映射来自 `generated/ti_msp_dl_config.h` 和 `pin_map.h`：

```text
KEY_1 -> PB9
KEY_2 -> PB8
```

关键变量：

```c
static volatile KeyEvent g_keyEvent = KEY_EVENT_NONE;
```

解释：

- `static`：只在 `key.c` 里可见，外面不能直接乱改。
- `volatile`：这个变量会在中断里被改，编译器不要自作聪明缓存它。
- `g_keyEvent`：最近一次按键事件。

中断来了以后：

```c
void Key_HandleGPIOInterrupt(void)
```

根据 pending interrupt 判断是 KEY1 还是 KEY2，然后把 `g_keyEvent` 改成 `KEY_EVENT_1` 或 `KEY_EVENT_2`。

主循环里调用：

```c
KeyEvent event = Key_PopEvent();
```

`Pop` 的意思是“取走并清空”。所以一个按键事件通常只处理一次。

## encoder.c：编码器计数

编码器有 A/B 两相信号。代码在 GPIOA 中断里读取 A/B 当前状态，用相位关系判断该加 1 还是减 1。

关键变量：

```c
static volatile int32_t g_leftCount;
static volatile int32_t g_rightCount;
```

主循环通过：

```c
Encoder_GetLeft();
Encoder_GetRight();
```

读取累计计数。速度闭环会用“本次计数 - 上次计数”得到一个周期内轮子转了多少。

## gray.c：7 路灰度传感器

灰度模块是本工程里最值得细看的模块之一。它负责：

1. 启动 ADC0/ADC1 采样。
2. 读取 7 路传感器的 ADC 原始值。
3. 连续采多次求平均，放到 `g_grayRaw[]`。
4. 用阈值把原始值变成黑/白数字量，放到 `g_grayDigital[]`。
5. 做连续确认防抖。
6. 计算循迹偏差。
7. 支持校准阈值。

### 为什么只有 ADC0 和 ADC1，不是 7 个 ADC

这里最容易误会。

不是只有两个灰度通道，而是芯片里用了两个 ADC 外设实例：`ADC0` 和 `ADC1`。每个 ADC 外设可以依次采多个输入通道，结果放到多个 MEM 槽。

这就像一个快递站有两个窗口，但可以处理七个包裹：

```text
传感器 S1 -> ADC0 -> MEM0
传感器 S2 -> ADC1 -> MEM0
传感器 S3 -> ADC1 -> MEM1
传感器 S4 -> ADC1 -> MEM2
传感器 S5 -> ADC1 -> MEM3
传感器 S6 -> ADC0 -> MEM1
传感器 S7 -> ADC0 -> MEM2
```

所以 `DL_ADC12_enableConversions(PIN_GRAY_ADC0);` 和 `DL_ADC12_enableConversions(PIN_GRAY_ADC1);` 是打开两个 ADC 外设，不是说只有两路传感器。

### GrayAdcSlot 是什么

代码：

```c
typedef struct {
    ADC12_Regs *adc;
    DL_ADC12_MEM_IDX mem;
} GrayAdcSlot;
```

这定义了一个结构体类型，名字叫 `GrayAdcSlot`。一个 `GrayAdcSlot` 变量里有两个成员：

- `adc`：指向哪个 ADC 外设，比如 ADC0 或 ADC1。
- `mem`：读这个 ADC 的哪个 MEM 结果槽。

`ADC12_Regs *adc` 里面的 `*` 是指针。你可以先理解成：它保存的是 ADC 外设寄存器区域的地址。TI 的 DriverLib 函数拿着这个地址，才能知道你要操作 ADC0 还是 ADC1。

### g_grayMap 和结构体有什么关系

代码：

```c
static const GrayAdcSlot g_grayMap[GRAY_SENSOR_COUNT] = {
    {PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY1},
    {PIN_GRAY_ADC1, GRAY_ADC1_MEM_GRAY2},
};
```

`g_grayMap` 是一个数组。数组的每个元素都是一个 `GrayAdcSlot` 结构体。

`{PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY1}` 没写 `.adc =`、`.mem =`，是因为 C 支持“按成员顺序初始化”：

```c
{PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY1}
```

等价于：

```c
{.adc = PIN_GRAY_ADC0, .mem = GRAY_ADC0_MEM_GRAY1}
```

所以后面读取时才可以写：

```c
DL_ADC12_getMemResult(g_grayMap[i].adc, g_grayMap[i].mem);
```

这个 `.adc`、`.mem` 出现在“使用结构体成员”的地方，不一定必须出现在初始化的地方。

### 为什么 ADC 是 0/1，MEM 名字像 1~7

`GRAY_ADC0_MEM_GRAY6` 这个名字里的 `GRAY6` 是“第 6 路灰度传感器”，不是说实际硬件 MEM6。

从生成文件看：

```text
GRAY_ADC0_MEM_GRAY1 -> DL_ADC12_MEM_IDX_0
GRAY_ADC0_MEM_GRAY6 -> DL_ADC12_MEM_IDX_1
GRAY_ADC0_MEM_GRAY7 -> DL_ADC12_MEM_IDX_2
GRAY_ADC1_MEM_GRAY2 -> DL_ADC12_MEM_IDX_0
GRAY_ADC1_MEM_GRAY3 -> DL_ADC12_MEM_IDX_1
GRAY_ADC1_MEM_GRAY4 -> DL_ADC12_MEM_IDX_2
GRAY_ADC1_MEM_GRAY5 -> DL_ADC12_MEM_IDX_3
```

也就是说，名字按传感器编号起，实际 MEM 槽在每个 ADC 内部从 0 开始排。

### g_grayRaw 是不是平均值

是。`g_grayRaw[]` 存的是最近一次 `Gray_Update()` 后的平均 ADC 值。

代码会采 `GRAY_FILTER_SAMPLE_COUNT` 次，把每一路加起来再除以采样次数：

```c
g_grayRaw[i] = sum[i] / GRAY_FILTER_SAMPLE_COUNT;
```

实际代码里还加了半个采样次数，是为了四舍五入：

```c
(sum[i] + (GRAY_FILTER_SAMPLE_COUNT / 2U)) / GRAY_FILTER_SAMPLE_COUNT
```

## oled.c：I2C OLED 和显存

OLED 地址是：

```c
#define OLED_I2C_ADDRESS (0x3CU)
```

最关键函数：

```c
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    uint8_t txData[2];
    txData[0] = mode ? 0x40 : 0x00;
    txData[1] = dat;
    ...
}
```

OLED I2C 发送时一次发两个字节：

```text
第 1 个字节：控制字节，告诉 OLED 后面的是命令还是显示数据
第 2 个字节：真正的内容 dat
```

`mode ? 0x40 : 0x00` 是 C 的三目运算符：

- 如果 `mode` 非 0，结果是 `0x40`，表示数据。
- 如果 `mode` 是 0，结果是 `0x00`，表示命令。

所以：

```c
OLED_WR_Byte(0xAF, OLED_CMD);
```

会发送：

```text
txData[0] = 0x00   告诉 OLED：后面是命令
txData[1] = 0xAF   告诉 OLED：打开显示
```

而显示字符时最终会发送：

```text
txData[0] = 0x40   告诉 OLED：后面是显示数据
txData[1] = 某个点阵字节
```

`OLED_GRAM[144][8]` 是屏幕缓冲区，也叫显存。很多画点、画字函数只是先改内存里的 `OLED_GRAM`，真正显示到屏幕上要调用 `OLED_Refresh()`。

## jy61p.c：姿态传感器

JY61P 用 UART0 发数据。代码按 11 字节一帧解析：

```text
帧头 0x55
帧类型 0x53 表示角度帧
后面数据里包含 yaw
最后 1 字节是校验和
```

中断收到字节后调用 `JY61P_ParseByte()` 拼帧。校验通过后，把 yaw 换算成整数角度，存到：

```c
static volatile int16_t g_jy61pYawDeg;
static volatile uint8_t g_jy61pYawValid;
```

路线模块 `route.c` 会用 yaw 判断直角转弯是否接近 90 度。

## jq8400.c：语音模块

JQ8400 当前代码很薄：

- `JQ8400_Init()` 打开 UART1 中断。
- `JQ8400_SendByte()` 阻塞发送 1 字节。
- `JQ8400_SendBytes()` 发送一段数据。

也就是说，它现在主要是“把 UART 通道准备好”，具体语音播放协议还没有写完整。

## link.c：调试/Exchange 串口

`link.c` 用 UART3 发送调试字符串。

常见调用：

```c
Link_SendString("state: tracking\r\n");
```

菜单监视页也会周期性通过 Link 打印灰度、编码器、PID 数据。这个模块当前主要是发送，接收解析还没有正式实现。

