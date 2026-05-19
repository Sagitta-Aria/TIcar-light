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
