# 04. hardware 传感器、显示和串口模块

本章讲相对复杂的硬件模块：

```text
hardware/gray.h
hardware/gray.c
hardware/oled.h
hardware/oled.c
hardware/oledfont.h
hardware/jy61p.h
hardware/jy61p.c
hardware/jq8400.h
hardware/jq8400.c
```

## gray.h：灰度模块接口

`GrayChannel` 枚举定义 7 路灰度传感器：

```c
GRAY_1 = 0,
GRAY_2,
...
GRAY_7
```

这 7 路按车头朝前时从左到右排列。

对外接口分几类：

```text
初始化和采样：
  Gray_Init()
  Gray_StartConversion()
  Gray_Update()

读取结果：
  Gray_GetRaw()
  Gray_GetDigital()
  Gray_GetDigitalMask()
  Gray_ReadRaw()
  Gray_ReadDigital()

循迹误差：
  Gray_GetLineError()
  Gray_GetWeightedLineError()
  Gray_IsLineLost()

阈值和校准：
  Gray_SetThreshold()
  Gray_GetThreshold()
  Gray_CalibrationReset()
  Gray_CalibrationSample()
  Gray_CalibrationApply()
```

## gray.c：灰度采样、滤波、阈值和误差

### GrayAdcSlot

```c
typedef struct {
    ADC12_Regs *adc;
    DL_ADC12_MEM_IDX mem;
} GrayAdcSlot;
```

一个灰度传感器读数需要两个信息：

```text
adc：属于 ADC0 还是 ADC1
mem：结果放在哪个 MEM 槽
```

所以用结构体把这两个信息绑在一起。

### g_grayMap

```c
static const GrayAdcSlot g_grayMap[GRAY_SENSOR_COUNT] = {
    {PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY1},
    {PIN_GRAY_ADC1, GRAY_ADC1_MEM_GRAY2},
    ...
};
```

这是“第几路灰度 -> 哪个 ADC + 哪个 MEM”的映射表。

为什么两个 ADC 能读七路？因为一个 ADC 外设可以按序列采多个输入通道。这里是：

```text
ADC0 负责 S1/S6/S7
ADC1 负责 S2/S3/S4/S5
```

### g_grayWeight

```c
static const int16_t g_grayWeight[GRAY_SENSOR_COUNT] = {
    -3, -2, -1, 0, 1, 2, 3
};
```

这是每路传感器的位置权重：

```text
左边为负
中间为 0
右边为正
```

后面计算循迹误差时会用它。

### g_grayRaw

```c
static uint16_t g_grayRaw[GRAY_SENSOR_COUNT];
```

这是“平均滤波后的 ADC 值”。不是单次采样值。

`Gray_Update()` 会采 `GRAY_FILTER_SAMPLE_COUNT` 次，然后求平均写入 `g_grayRaw[]`。

### g_grayDigital 和阈值

```c
g_grayDigital[i] = Gray_RawToDigital(g_grayRaw[i], g_grayThreshold[i]);
```

灰度原始值是 0~4095 的 ADC 数字。循迹时经常还需要一个简单黑白结果：

```text
0：未压线
1：压线
```

`GRAY_ACTIVE_HIGH` 决定“ADC 越大越像黑线”还是相反。

### Gray_Update()

这是灰度模块主函数，流程是：

```text
sum[] 清零
重复采样 N 次：
  Gray_ReadRawOnce(rawOnce)
  每一路累加到 sum[]
每一路求平均，写入 g_grayRaw[]
Gray_UpdateDigitalFromRaw()
g_grayValid = 1
```

如果 ADC 等待超时，会返回 0，表示这次更新失败。

### Gray_UpdateDigitalFromRaw()

这个函数做“原始值转黑白”和“防抖”。

第一次采样时直接接受当前状态。后续如果状态要变化，需要连续出现 `GRAY_DIGITAL_CONFIRM_COUNT` 次才确认。

目的：避免 ADC 抖动导致黑白状态频繁跳。

### Gray_GetWeightedLineError()

正式循迹更常用这个函数。

它不是只看黑白数量，而是用压线强度做加权平均：

```text
weightedSum = 位置权重 * 该路强度
strengthSum = 所有强度之和
error = weightedSum / strengthSum
```

输出含义：

```text
负数：线偏左
0 附近：线在中间
正数：线偏右
```

### 校准流程

校准三步：

```text
Gray_CalibrationReset()
Gray_CalibrationSample() 反复调用，记录每路 min/max
Gray_CalibrationApply() 取 min/max 中点作为阈值
```

菜单里的 `1.Calib` 就是走这套逻辑。

## oled.h：OLED 对外接口

`oled.h` 里有一些旧式类型：

```c
typedef unsigned char u8;
typedef unsigned int u32;
```

现在更推荐 `uint8_t`、`uint32_t`，但旧 OLED 例程常常这样写。

核心宏：

```c
#define OLED_CMD  0
#define OLED_DATA 1
```

这对应 `OLED_WR_Byte(dat, mode)` 的第二个参数：

```text
OLED_CMD：dat 是命令
OLED_DATA：dat 是显示数据
```

## oled.c：I2C OLED 和显存

### OLED_GRAM

```c
u8 OLED_GRAM[144][8];
```

这是屏幕显存缓冲区。OLED 是 128x64 像素，竖直方向按 page 分成 8 页，每页 8 像素高。

很多函数先改 `OLED_GRAM`，再由 `OLED_Refresh()` 统一发到屏幕。

### OLED_WR_Byte()

这是最核心的 I2C 写函数。

```c
uint8_t txData[2];
txData[0] = mode ? 0x40 : 0x00;
txData[1] = dat;
```

OLED I2C 写入两个字节：

```text
第 1 字节：控制字节
  0x00 表示后面是命令
  0x40 表示后面是显示数据

第 2 字节：真正的 dat
```

例如：

```c
OLED_WR_Byte(0xAF, OLED_CMD);
```

意思是发送命令 `0xAF`，点亮屏幕。

### OLED_DisPlay_On()

```c
OLED_WR_Byte(0x8D, OLED_CMD);
OLED_WR_Byte(0x14, OLED_CMD);
OLED_WR_Byte(0xAF, OLED_CMD);
```

这三条都是 OLED 控制命令：

```text
0x8D：选择电荷泵设置
0x14：开启电荷泵
0xAF：打开显示
```

### OLED_Refresh()

OLED 屏分 8 页刷新：

```text
for page 0..7:
  设置页地址
  设置列地址
  发 128 个数据字节
```

所以画字、画点以后如果不刷新，屏幕不一定马上变化。

### OLED_ShowChar() / OLED_ShowString()

`OLED_ShowChar()` 从 `oledfont.h` 的 ASCII 字模数组里取点阵数据，然后逐位画点。

`OLED_ShowString()` 循环显示每个字符，超出一行会换行。

本工程菜单用短英文，是因为这套 ASCII 字库显示英文更直接，中文需要额外取模和字库索引。

### OLED_Init()

OLED 初始化发送一串 SSD1306 类命令，包括：

- 关闭显示。
- 设置列地址、起始行。
- 设置对比度。
- 设置扫描方向。
- 设置复用率。
- 设置时钟。
- 开启电荷泵。
- 打开显示。
- 清屏。

你现在不用背每个命令，只要知道它们是在配置 OLED 控制芯片的显示参数。

## oledfont.h：字库文件

这个文件主要是数组，保存点阵字体。

比如：

```c
const unsigned char asc2_1206[95][12]
```

意思是：95 个 ASCII 可显示字符，每个字符 12 字节点阵数据。

这类文件不用逐行读。你只要知道：

```text
OLED_ShowChar() 根据字符编号从这里取点阵
点阵里的每一位决定某个像素亮不亮
```

## jy61p.h / jy61p.c：JY61P 姿态模块

JY61P 用 UART0 收数据。

### 帧格式

代码里定义：

```c
#define JY61P_FRAME_HEAD   (0x55U)
#define JY61P_FRAME_ANGLE  (0x53U)
#define JY61P_FRAME_LENGTH (11U)
```

意思是 JY61P 每帧 11 字节，角度帧类型是 `0x53`，帧头是 `0x55`。

### 全局缓存

```c
static volatile int16_t g_jy61pYawDeg;
static volatile uint8_t g_jy61pYawValid;
```

解析到有效 yaw 后写入这里。路线模块用：

```c
JY61P_GetYawDeg()
JY61P_HasYaw()
```

读取当前角度。

### JY61P_ParseByte()

UART 中断每收到一个字节，就喂给解析函数。

它做的事情：

```text
等帧头 0x55
收满 11 字节
检查帧类型是不是 0x53
检查校验和
提取 yaw
换算成角度
```

这种“一个字节一个字节拼帧”的写法，是串口协议里很常见的状态机思路。

### JY61P_HandleUARTInterrupt()

这是 UART0 中断处理。它不断从 UART RX FIFO 里取字节，然后调用 `JY61P_ParseByte()`。

## jq8400.h / jq8400.c：语音模块

JQ8400 用 UART1。

当前代码只有基础发送接口：

```c
JQ8400_SendByte()
JQ8400_SendBytes()
```

还没有完整写“播放第几首、设置音量、停止播放”等协议命令。

所以你读到这里时可以先把它理解成：

```text
UART1 通道已经准备好，后续要按 JQ8400 协议继续封装命令
```

