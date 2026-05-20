# 05. 调试现象和代码逻辑对照

这一页专门放你前面问过的现象：BSL、RESET、OLED、按键、下载失败。这里只解释代码和单片机概念，不改工程代码。

## BSL 按键是什么

BSL 通常指 Boot Strap Loader，也就是芯片内置的启动加载器入口。

它不是普通意义上的“OLED 使能键”。更准确地说，它可能影响芯片复位后进入用户程序，还是进入芯片 ROM 里的下载/引导模式。不同开发板的 BSL 按键接法不完全一样，但常见用法是：按住 BSL，再复位，让芯片进入引导下载模式。

所以如果你按 BSL 后 OLED 亮了，不要先理解成“BSL 使能 OLED”。更可能是：

- BSL 按键改变了启动/复位时序。
- 按键动作让芯片重新启动了一次。
- 当前程序在某个阶段初始化了 OLED，但后续又卡住或清屏。
- 当前 main 如果是空循环，正常应用逻辑不会跑。

## RESET 和 BSL 的区别

RESET 是复位：让芯片从头开始跑。

BSL 是启动模式选择相关：可能让芯片进入 bootloader，而不是正常用户程序。

简单记：

```text
RESET：重新开始跑
BSL：决定从哪里/以什么模式开始跑
```

## 上电 OLED 闪一下又灭，可能看哪里

只从代码逻辑看，常见检查顺序：

1. `main.c` 里 `Board_Init()`、`App_Init()`、`App_Task()` 是否真的在跑。
2. `Board_Init()` 是否卡在时钟、I2C、UART、ADC 某一步。
3. OLED 是否被初始化后又被 `OLED_Clear()` 清掉，而后续菜单没有刷新。
4. I2C 是否因为等待 BUSY/IDLE 状态卡在 `OLED_WR_Byte()`。
5. 供电是否稳定，OLED 上电复位时间是否够。
6. 下载进去的程序是否真的是你以为的那份。

当前 `board.c` 里有启动进度页：

```text
CLK...
I2C OK
UART WAIT
ADC WAIT
APP WAIT
```

如果屏幕能停在某一行，就可以判断卡在哪一步。如果只是闪一下就没了，可能是复位、清屏、I2C 卡住、程序没跑完整，或者实际下载失败。

## 代码里按键对 OLED 的控制逻辑

按键本身不直接控制 OLED 开关。按键控制的是菜单和状态机。

路径是：

```text
PB9/PB8 电平变化
  -> GPIOB_IRQHandler()
  -> Key_HandleGPIOInterrupt()
  -> g_keyEvent = KEY_EVENT_1/2
  -> App_Task()
  -> App_HandleKeyEvent(Key_PopEvent())
  -> Menu_Next() 或 StateMachine_Dispatch(...)
  -> Menu_Task()
  -> OLED_ShowLine()/OLED_Refresh()
```

所以如果按键“没有用”，要分开判断：

- PB8/PB9 电平有没有变化。
- GPIOB 中断有没有进。
- `g_keyEvent` 有没有被设置。
- `App_Task()` 有没有在主循环里跑。
- 当前状态下 KEY1/KEY2 是否有对应动作。
- OLED 刷新是否正常。

如果 `App_Task()` 没有跑，即使按键中断设置了事件，也没有主循环去消费这个事件，菜单就不会变。

## 下载失败可能看哪里

下载失败通常不一定是 C 代码问题，更多是工具链、调试器、接线、芯片状态。

可以按这个顺序排：

1. J-Link 或仿真器是否被电脑识别。
2. Keil 里选的芯片型号、Debug adapter、下载算法是否正确。
3. SWDIO/SWCLK/GND/3V3 是否接对且接触稳定。
4. 板子是否供电稳定。
5. 芯片是否被 BSL/低功耗/复位脚状态影响。
6. 当前工程文件 `.uvprojx` 是否仍是正确配置。
7. Keil 报错原文是什么，尤其是 connect failed、no device found、flash timeout、verify failed 这类关键词。

如果下载失败，不要急着改业务代码。先把错误原文、截图、Keil Output 窗口内容保存下来，再判断是连接问题、下载算法问题，还是程序上电后把调试口影响了。

## 当前调试时最容易混淆的点

OLED 亮不亮，不等于主程序完整运行。

按键中断能触发，不等于主循环有处理。

BSL 让 OLED 亮，不等于 BSL 是 OLED 电源开关。

ADC0/ADC1 只有两个，不等于灰度传感器只有两个。

`g_grayRaw[]` 是平均后的 ADC 数组，不是单次原始采样。

`mode` 是函数参数，不是 `txData` 数组。

结构体初始化时没写 `.adc`，不代表没有结构体成员，只是按顺序填了。

