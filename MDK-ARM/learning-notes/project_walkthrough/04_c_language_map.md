# 04. 这份工程里最常见的 C 语言知识

这份不是完整 C 教程，而是专门解释你在本工程里会反复撞见的语法。

## `0U` 是什么

`0U` 就是无符号整数 0。

- `0`：普通 int 类型的 0。
- `0U`：unsigned int 类型的 0。

`U` 的意思是 `unsigned`。

在单片机代码里经常写 `0U`、`1U`、`1000U`，原因是很多寄存器、计数器、长度、数组下标都用无符号类型，比如 `uint8_t`、`uint16_t`、`uint32_t`。这样写可以减少有符号/无符号混用带来的编译警告。

例子：

```c
uint32_t i;
for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
}
```

这里 `i` 是 `uint32_t`，所以初始值也写成 `0U`。

## `uint8_t`、`uint16_t`、`int16_t`

这些来自 `<stdint.h>`，好处是位数明确。

```text
uint8_t   无符号 8 位整数，范围 0~255
uint16_t  无符号 16 位整数，范围 0~65535
uint32_t  无符号 32 位整数
int16_t   有符号 16 位整数，范围 -32768~32767
int32_t   有符号 32 位整数
```

单片机里寄存器、ADC 值、PWM 值都很在意位数，所以常用这些类型。

## `typedef struct`：给结构体类型起名字

灰度代码：

```c
typedef struct {
    ADC12_Regs *adc;
    DL_ADC12_MEM_IDX mem;
} GrayAdcSlot;
```

这句话定义了一个新类型 `GrayAdcSlot`。

以后就可以写：

```c
GrayAdcSlot slot;
```

不用写很长的 `struct xxx`。

结构体像一个小表格，一个变量里装多个相关字段：

```text
GrayAdcSlot
  adc: 哪个 ADC 外设
  mem: 哪个 ADC 结果槽
```

## 结构体数组

```c
static const GrayAdcSlot g_grayMap[GRAY_SENSOR_COUNT] = {
    {PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY1},
    {PIN_GRAY_ADC1, GRAY_ADC1_MEM_GRAY2},
};
```

这不是定义两个结构体类型，而是定义一个数组。数组名是 `g_grayMap`，数组里每个元素都是 `GrayAdcSlot`。

可以这样理解：

```text
g_grayMap[0] = { adc: PIN_GRAY_ADC0, mem: GRAY_ADC0_MEM_GRAY1 }
g_grayMap[1] = { adc: PIN_GRAY_ADC1, mem: GRAY_ADC1_MEM_GRAY2 }
...
```

读取时：

```c
g_grayMap[i].adc
g_grayMap[i].mem
```

`.` 是取结构体成员。

## 为什么初始化时没写 `.adc`

C 语言结构体可以按顺序初始化。

结构体定义顺序是：

```c
ADC12_Regs *adc;
DL_ADC12_MEM_IDX mem;
```

所以：

```c
{PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY1}
```

第一个值给 `adc`，第二个值给 `mem`。

如果写完整一点就是：

```c
{.adc = PIN_GRAY_ADC0, .mem = GRAY_ADC0_MEM_GRAY1}
```

两种写法都可以。项目里用了更短的按顺序写法。

## `*`：指针

`ADC12_Regs *adc` 里的 `*` 表示 `adc` 是一个指针。

初学时先这样理解：

- 普通变量保存“值”。
- 指针变量保存“地址”。

单片机外设寄存器都在固定地址上。`ADC12_Regs *adc` 保存的是某个 ADC 外设寄存器区域的地址。TI 的函数拿到这个地址后，就知道该操作哪个 ADC。

比如：

```c
DL_ADC12_getMemResult(g_grayMap[i].adc, g_grayMap[i].mem);
```

第一个参数告诉它读 ADC0 还是 ADC1；第二个参数告诉它读哪个 MEM 结果槽。

## `&`：取地址

速度闭环里有：

```c
SpeedControl_StopWheel(&g_leftWheel, Encoder_GetLeft());
```

`&g_leftWheel` 表示把 `g_leftWheel` 这个结构体变量的地址传进去。

为什么要传地址？因为函数内部要修改这个结构体。如果只传值，函数拿到的是拷贝，改完外面的变量不会变。

## `->`：通过指针访问结构体成员

速度闭环里：

```c
wheel->targetCommand = command;
```

`wheel` 是结构体指针。`wheel->targetCommand` 等价于：

```c
(*wheel).targetCommand
```

意思是：先通过指针找到那个结构体，再访问里面的 `targetCommand` 成员。

## `static`

`static` 在本工程里常见两种用法。

用于全局变量：

```c
static uint8_t g_grayMask;
```

意思是这个变量只在当前 `.c` 文件里可见，外部文件不能直接访问。

用于函数：

```c
static void Gray_RebuildMask(void)
```

意思是这个函数只给当前 `.c` 文件内部使用，不暴露给别的文件。

这是一种保护边界的写法：外部只能调用 `.h` 里声明的公共接口。

## `const`

```c
static const int16_t g_grayWeight[GRAY_SENSOR_COUNT] = {
    -3, -2, -1, 0, 1, 2, 3
};
```

`const` 表示只读，不应该被修改。这里每个灰度传感器的位置权重是固定配置，不需要运行时改。

## `volatile`

```c
static volatile KeyEvent g_keyEvent;
static volatile int32_t g_leftCount;
```

`volatile` 常用于中断会修改的变量。

编译器优化时可能会觉得：“这个变量主循环里没改，那我就不用每次从内存读了。”但中断可能随时改它，所以要加 `volatile` 告诉编译器：每次都老老实实从内存读。

## `enum`：枚举

```c
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_1,
    KEY_EVENT_2
} KeyEvent;
```

枚举就是给一组整数起名字。

实际值大概是：

```text
KEY_EVENT_NONE = 0
KEY_EVENT_1    = 1
KEY_EVENT_2    = 2
```

好处是代码更像人话：

```c
if (event == KEY_EVENT_1)
```

比写：

```c
if (event == 1)
```

清楚得多。

## 宏 `#define`

```c
#define GRAY_SENSOR_COUNT (7U)
```

宏是预处理阶段的文本替换。编译前，代码里看到 `GRAY_SENSOR_COUNT`，会替换成 `(7U)`。

项目里把调参值放进宏，方便统一修改。

## 条件编译 `#if`

```c
#if CAR_ENABLE_SPEED_CONTROL
    SpeedControl_SetTarget(left, right);
#else
    Motor_SetSpeed(left, right);
#endif
```

这是编译前决定保留哪段代码。`CAR_ENABLE_SPEED_CONTROL` 为 1 时编译闭环版本，为 0 时编译开环版本。

它不是运行时 if，而是编译时选择。

## 三目运算符 `? :`

OLED 代码：

```c
txData[0] = mode ? 0x40 : 0x00;
```

格式：

```c
条件 ? 条件为真时的值 : 条件为假时的值
```

所以这句等价于：

```c
if (mode) {
    txData[0] = 0x40;
} else {
    txData[0] = 0x00;
}
```

`mode` 不是数组。这里数组是 `txData`，`mode` 是函数参数，用来决定当前写的是 OLED 命令还是 OLED 数据。

## 位运算

单片机代码经常用位运算，因为寄存器和 GPIO 都是一位一位控制。

常见符号：

```text
&   按位与，用来判断某一位是不是 1
|   按位或，用来把某一位置 1
~   按位取反
<<  左移
>>  右移
```

灰度 mask：

```c
g_grayMask |= (uint8_t)(1U << i);
```

如果第 `i` 路灰度为 1，就把 `g_grayMask` 的第 `i` 位置 1。

比如：

```text
i = 0 -> 1U << 0 -> 00000001
i = 3 -> 1U << 3 -> 00001000
```

判断某个引脚：

```c
(DL_GPIO_readPins(PIN_KEY_PORT, pin) & pin) ? 1U : 0U
```

`& pin` 的意思是只关心对应引脚那一位。

## `NULL` 和 `0`

代码里有时写：

```c
if (values == 0) {
    return;
}
```

这里 `values` 是数组参数，本质上会变成指针。判断 `values == 0` 就是在判断这个指针是不是空指针。

有些地方写 `NULL`，有些地方写 `0`，意思接近。初学时你可以理解为：如果指针是空的，就不要继续访问，避免程序跑飞。

## 函数参数是“输入”，数组是“容器”

OLED 例子：

```c
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    uint8_t txData[2];
    txData[0] = mode ? 0x40 : 0x00;
    txData[1] = dat;
}
```

这里：

- `dat`：调用者传进来的一个字节内容。
- `mode`：调用者传进来的模式，命令或数据。
- `txData[2]`：函数内部临时创建的数组，用来装准备通过 I2C 发出去的两个字节。

调用：

```c
OLED_WR_Byte(0xAF, OLED_CMD);
```

进入函数后：

```text
dat  = 0xAF
mode = OLED_CMD，也就是 0
txData[0] = 0x00
txData[1] = 0xAF
```

所以 `mode` 不是你给的数组，它只是决定数组第 0 个字节填什么。

