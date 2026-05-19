# OLED I2C 代码讲解

更新时间：2026-05-19

这份笔记记录 `hardware/oled.c` 里 OLED 通过 I2C 写命令和写数据的关键代码。

## 1. `txData[2]` 是什么

代码：

```c
uint8_t txData[2];
```

意思是定义一个长度为 2 的 `uint8_t` 数组。

```text
txData[0] -> 第 1 个要发出去的字节
txData[1] -> 第 2 个要发出去的字节
```

这里每次通过 I2C 给 OLED 发 2 个字节：

```text
第 1 个字节：控制字节，告诉 OLED 后面的是命令还是显示数据
第 2 个字节：真正要发的内容，也就是 dat
```

## 2. `mode ? 0x40 : 0x00` 是什么

代码：

```c
txData[0] = mode ? 0x40 : 0x00;
```

这是 C 语言的三目运算符：

```c
条件 ? 条件成立时的值 : 条件不成立时的值
```

所以这句等价于：

```c
if (mode) {
    txData[0] = 0x40;
} else {
    txData[0] = 0x00;
}
```

也就是说：

```text
mode 不是 0 -> txData[0] = 0x40
mode 是 0   -> txData[0] = 0x00
```

## 3. 为什么 `0x00` 是命令，`0x40` 是数据

OLED 控制器通过 I2C 接收内容时，需要先收到一个“控制字节”。

在这份代码里：

```text
0x00 表示后面的 dat 是命令
0x40 表示后面的 dat 是显示数据
```

比如：

```c
OLED_WR_Byte(0xAE, OLED_CMD);
```

含义是：

```text
发控制字节 0x00
再发命令 0xAE
```

`0xAE` 是 OLED 关闭显示的命令。

再比如：

```c
OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
```

含义是：

```text
发控制字节 0x40
再发一个屏幕显示数据字节
```

## 4. `txData[1] = dat` 是什么

代码：

```c
txData[1] = dat;
```

意思是把函数参数 `dat` 放到第二个发送字节里。

`dat` 本身可能是两类东西：

- 如果 `mode` 是 `OLED_CMD`，`dat` 就是 OLED 命令
- 如果 `mode` 是 `OLED_DATA`，`dat` 就是屏幕显示数据

所以完整发送格式是：

```text
txData[0] = 控制字节
txData[1] = 真正内容
```

## 5. 整个函数这一段可以这样读

```c
uint8_t txData[2];

txData[0] = mode ? 0x40 : 0x00;
txData[1] = dat;
```

人话版：

```text
准备两个字节。
第一个字节告诉 OLED：我后面发的是命令还是显示数据。
第二个字节才是真正要发给 OLED 的内容。
```

## 6. 一句话记忆

```text
I2C 给 OLED 发东西时，先发“这是什么”，再发“内容是什么”。
```

## 7. 用 `OLED_DisPlay_On()` 对照理解

代码：

```c
void OLED_DisPlay_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD); /* 电荷泵使能 */
    OLED_WR_Byte(0x14, OLED_CMD); /* 开启电荷泵 */
    OLED_WR_Byte(0xAF, OLED_CMD); /* 点亮屏幕 */
}
```

`OLED_WR_Byte()` 的函数定义是：

```c
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
```

所以调用时参数是这样对应的：

```text
OLED_WR_Byte(0x8D, OLED_CMD)
              |      |
              |      +-> mode
              +--------> dat
```

`mode` 不是数组。`mode` 是函数的第二个参数，用来表示这次写的是命令还是数据。

在 `oled.h` 里：

```c
#define OLED_CMD  0
#define OLED_DATA 1
```

所以：

```c
OLED_WR_Byte(0x8D, OLED_CMD);
```

等价于：

```c
OLED_WR_Byte(0x8D, 0);
```

进入函数内部以后：

```c
dat  = 0x8D
mode = 0
```

于是：

```c
txData[0] = mode ? 0x40 : 0x00;
txData[1] = dat;
```

会变成：

```c
txData[0] = 0x00;
txData[1] = 0x8D;
```

也就是说，真正通过 I2C 发出去的是：

```text
0x00 0x8D
```

三行完整展开就是：

| 原始调用 | dat | mode | 实际发送的两个字节 |
| --- | --- | --- | --- |
| `OLED_WR_Byte(0x8D, OLED_CMD)` | `0x8D` | `0` | `0x00 0x8D` |
| `OLED_WR_Byte(0x14, OLED_CMD)` | `0x14` | `0` | `0x00 0x14` |
| `OLED_WR_Byte(0xAF, OLED_CMD)` | `0xAF` | `0` | `0x00 0xAF` |

第一个字节 `0x00` 告诉 OLED：

```text
后面这个字节是命令
```

第二个字节才是真正的命令内容：

```text
0x8D -> 准备配置电荷泵
0x14 -> 开启电荷泵
0xAF -> 打开显示
```
