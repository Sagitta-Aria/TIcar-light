# 灰度 ADC 代码讲解

更新时间：2026-05-19

这份笔记只讲 `hardware/gray.c` 里和灰度传感器有关的核心点，方便你以后继续往这里加新理解，不把 `README.md` 塞满。

## 1. `GrayAdcSlot` 是什么

```c
typedef struct {
    ADC12_Regs *adc;
    DL_ADC12_MEM_IDX mem;
} GrayAdcSlot;
```

它可以理解成“一个灰度传感器采样位置的说明书”。

- `typedef`：给类型起别名，后面就能直接写 `GrayAdcSlot`，不用每次写 `struct ...`
- `struct`：结构体，里面可以装多个字段
- `ADC12_Regs *adc`：这个通道属于哪个 ADC 外设
- `DL_ADC12_MEM_IDX mem`：这个结果存到 ADC 的哪个 MEM 槽位

你可以把它想成：

```text
传感器第几路  ->  用哪个 ADC  ->  存到哪个结果槽
```

比如：

```c
{PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY1}
```

意思就是：

```text
灰度 1 路，挂在 ADC0 的 MEM0 上
```

### 这里的 `*` 是什么

`*` 在声明里表示“指针”。

```c
ADC12_Regs *adc;
```

意思不是“一个 ADC”，而是“一个指向 ADC12 寄存器块的地址”。

更直白一点：

- `ADC12_Regs`：TI 设备给 ADC 外设做的寄存器结构体
- `*adc`：这个变量里装的是 ADC 外设寄存器的地址

所以这里不是“ADC12 路口”，而是“ADC 外设寄存器映射”。

你可以把它理解成：

```text
adc 不是实物，是指向硬件寄存器那块内存的门牌号
```

## 2. 为什么到处都是 `0U`

`0U` 的意思就是“无符号的 0”。

```c
uint32_t i;
for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
}
```

这里写 `0U` 有几个好处：

- 和 `uint32_t`、`uint16_t` 这种无符号类型更匹配
- 少一些编译器的 signed/unsigned 警告
- 代码一眼就能看出：这个数是给无符号场景用的

`0` 和 `0U` 数值上都等于 0，但 `U` 是类型提示。

你在这份代码里看到的 `1U`、`0U`、`100000U` 也是同一类写法。

## 3. 是不是只有两个 ADC 通道

不是。

这块板子里是：

- **2 个 ADC 外设实例**：`ADC0` 和 `ADC1`
- **7 路灰度传感器采样结果**
- **多个 MEM 槽位** 来存这 7 路数据

对应关系在 `g_grayMap` 里：

```c
static const GrayAdcSlot g_grayMap[GRAY_SENSOR_COUNT] = {
    {PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY1},
    {PIN_GRAY_ADC1, GRAY_ADC1_MEM_GRAY2},
    {PIN_GRAY_ADC1, GRAY_ADC1_MEM_GRAY3},
    {PIN_GRAY_ADC1, GRAY_ADC1_MEM_GRAY4},
    {PIN_GRAY_ADC1, GRAY_ADC1_MEM_GRAY5},
    {PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY6},
    {PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY7},
};
```

这个数组和结构体的关系是：

```text
g_grayMap 是一个数组
数组里的每一项，都是一个 GrayAdcSlot 结构体
每个 GrayAdcSlot 结构体里有 adc 和 mem 两个成员
```

你没看到 `.adc`、`.mem`，是因为这里用了 C 语言的“按成员顺序初始化”。

结构体定义顺序是：

```c
typedef struct {
    ADC12_Regs *adc;      /* 第 1 个成员 */
    DL_ADC12_MEM_IDX mem; /* 第 2 个成员 */
} GrayAdcSlot;
```

所以这一行：

```c
{PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY1}
```

等价于：

```c
{
    .adc = PIN_GRAY_ADC0,
    .mem = GRAY_ADC0_MEM_GRAY1
}
```

也就是：

```text
第 1 个值填进 adc
第 2 个值填进 mem
```

这两种写法都可以。原代码用的是更短的写法，适合这种字段很少、顺序很清楚的映射表。

### 为什么 ADC 只有 0/1，MEM 名字却像 1~7

这里最容易误会的是宏名：

```c
GRAY_ADC0_MEM_GRAY1
GRAY_ADC1_MEM_GRAY2
GRAY_ADC1_MEM_GRAY3
```

名字里的 `GRAY1`、`GRAY2`、`GRAY3` 指的是“第几路灰度传感器”，不是实际的 MEM 编号。

真实 MEM 编号要看 `generated/ti_msp_dl_config.h`：

```c
#define GRAY_ADC0_MEM_GRAY1  DL_ADC12_MEM_IDX_0
#define GRAY_ADC0_MEM_GRAY6  DL_ADC12_MEM_IDX_1
#define GRAY_ADC0_MEM_GRAY7  DL_ADC12_MEM_IDX_2
#define GRAY_ADC1_MEM_GRAY2  DL_ADC12_MEM_IDX_0
#define GRAY_ADC1_MEM_GRAY3  DL_ADC12_MEM_IDX_1
#define GRAY_ADC1_MEM_GRAY4  DL_ADC12_MEM_IDX_2
#define GRAY_ADC1_MEM_GRAY5  DL_ADC12_MEM_IDX_3
```

所以实际关系是：

| 灰度传感器 | 使用 ADC | 实际 MEM 槽 |
| --- | --- | --- |
| 灰度 1 | ADC0 | MEM0 |
| 灰度 2 | ADC1 | MEM0 |
| 灰度 3 | ADC1 | MEM1 |
| 灰度 4 | ADC1 | MEM2 |
| 灰度 5 | ADC1 | MEM3 |
| 灰度 6 | ADC0 | MEM1 |
| 灰度 7 | ADC0 | MEM2 |

也就是说：

```text
ADC0/ADC1 是硬件 ADC 外设编号
GRAY1~GRAY7 是灰度传感器编号
MEM0/MEM1/MEM2... 是每个 ADC 自己的结果槽编号
```

MEM 编号不是全局从 1 到 7 排，而是每个 ADC 内部自己从 `MEM0` 开始排。

也就是说：

- 左边第 1 路用 ADC0
- 中间几路用 ADC1
- 右边几路又回到 ADC0

所以 `DL_ADC12_enableConversions(PIN_GRAY_ADC0);` 和 `DL_ADC12_enableConversions(PIN_GRAY_ADC1);` 的意思是：

```text
把这两个 ADC 外设都打开，准备让它们跑采样序列
```

它不是“只有两个传感器”。

### 为什么不是实例化 7 个 ADC

因为“ADC 外设”和“ADC 输入通道”不是一回事。

灰度传感器有 7 路，意思是有 7 根模拟信号线。可是 MSPM0 芯片里面真正负责模数转换的硬件 ADC 模块没有 7 个，而是用 `ADC0`、`ADC1` 这类外设去轮流采多路输入。

可以这样类比：

```text
7 个学生交作业  ->  2 个老师批改  ->  每份作业放到不同收件格
7 路灰度信号    ->  2 个 ADC 外设  ->  每路结果放到不同 MEM 槽
```

所以程序里只需要保存：

```text
这一传感器用哪个 ADC
这一传感器的结果在哪个 MEM
```

也就是：

```c
typedef struct {
    ADC12_Regs *adc;
    DL_ADC12_MEM_IDX mem;
} GrayAdcSlot;
```

如果真的“实例化 7 个 ADC”，那含义会变成芯片里有 7 套完整 ADC 硬件。实际不是这样。实际是：

- `ADC0` 是一套 ADC 硬件
- `ADC1` 是另一套 ADC 硬件
- 每套 ADC 可以配置多个采样输入和多个 MEM 结果槽
- 7 路灰度只是被分配到这两套 ADC 硬件上

因此 `g_grayMap` 不是在创建 ADC，而是在记录映射关系：

```text
灰度 1 -> ADC0 的 MEM0
灰度 2 -> ADC1 的 MEM0
灰度 3 -> ADC1 的 MEM1
灰度 4 -> ADC1 的 MEM2
灰度 5 -> ADC1 的 MEM3
灰度 6 -> ADC0 的 MEM1
灰度 7 -> ADC0 的 MEM2
```

这里的“实例”更接近硬件外设实例，不是 C++ 那种你想 new 几个对象就 new 几个对象。单片机里很多外设数量是芯片出厂就固定好的，软件只能使用和配置它们。

## 4. `g_grayRaw` 是不是平均值

是，但更准确地说是：

```text
多次采样后的平均原始 ADC 值
```

在 `Gray_Update()` 里，它会先采样多次，再求平均：

```c
g_grayRaw[i] =
    (uint16_t)((sum[i] + (GRAY_FILTER_SAMPLE_COUNT / 2U)) /
        GRAY_FILTER_SAMPLE_COUNT);
```

它的含义是：

- 还没做黑白判断
- 还是 ADC 原始量
- 只是先做了滤波平均，让数据更稳

所以它不是“最终黑线/白线结果”，而是“更平滑的原始值”。

## 5. `DL_ADC12_enableConversions(...)` 在干什么

```c
DL_ADC12_enableConversions(PIN_GRAY_ADC0);
DL_ADC12_enableConversions(PIN_GRAY_ADC1);
```

这里是在启用 ADC 转换功能。

你可以把流程想成：

```text
先开电源
再允许 ADC 跑起来
最后才 startConversion
```

常见顺序是：

1. `enableConversions(...)`
2. `startConversion(...)`
3. 等待完成
4. 读取 MEM 结果

## 6. 这份灰度代码的完整数据流

```text
原始 ADC
  -> 多次采样求平均
  -> g_grayRaw
  -> 和阈值比较
  -> g_grayDigital
  -> 拼成 g_grayMask
  -> 算偏差 error
```

你可以重点盯住这几个变量：

- `g_grayRaw`：平均后的原始值
- `g_grayThreshold`：阈值
- `g_grayDigital`：0/1 黑白状态
- `g_grayMask`：7 路压成一个 bit 图

## 7. 读这类代码的顺序

建议你按这个顺序看：

1. `gray.h`，先看对外能调用什么
2. `Gray_Init()`，看初始化干了什么
3. `Gray_Update()`，看数据怎么进来
4. `Gray_RawToDigital()`，看怎么分黑白
5. `Gray_GetWeightedLineError()`，看怎么给小车控制用

## 8. 你现在可以先记住的一句话

这段代码不是“在读 7 个数字”，而是：

```text
把 7 路灰度传感器的模拟电压，稳定地变成小车能用的偏差值
```

---

下一步你要是继续看 `gray.c`，重点就看：

- 为什么要做平均滤波
- 为什么要做连续确认防抖
- `g_grayWeight` 怎么影响转向
- `Gray_GetWeightedLineError()` 为什么比 `Gray_GetLineError()` 更适合正式循迹
