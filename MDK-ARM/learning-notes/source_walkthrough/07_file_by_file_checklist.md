# 07. 全工程逐文件速查

这一页按目录列出每个源码/文档文件的作用。读完前面几章后，可以用它快速定位。

## 根目录

### README.md

项目介绍和使用入口。它说明目录结构、构建方式、菜单、状态机、路线、速度闭环等概要。

README 适合“别人第一次打开项目先看”，不适合放很长的学习笔记，所以详细讲解放在 `MDK-ARM/learning-notes/`。

## doc

### doc/PINMAP.md

引脚和外设连接表。读任何硬件代码前都应该先看它。

重点：

```text
OLED: I2C0 PA0/PA1
JY61P: UART0
JQ8400: UART1
Exchange: UART3
电机: TIMA0 PWM + GPIO 方向
灰度: 7 路 ADC
按键: PB9/PB8
编码器: PA12/PA13/PA22/PA23
```

## config

### config/pin_map.h

项目引脚别名表。把 generated 里的宏换成 `PIN_` 开头的项目语义名。

读法：看到 `PIN_GRAY_ADC0`，去这里查它实际等于 `GRAY_ADC0_INST`。

### config/board_config.h

全局调参表。包含电机 PWM 上限、循迹参数、丢线处理、菜单刷新周期、速度闭环、路线外环、灰度阈值等。

调车优先看这里。

## generated

### generated/ti_msp_dl_config.h

TI DriverLib/SysConfig 风格的配置头文件。定义芯片型号、CPU 频率、外设实例、引脚、MEM 槽、初始化函数声明。

不建议初学者一开始逐行背，但需要会查外设和引脚。

### generated/ti_msp_dl_config.c

底层外设初始化实现。配置电源、GPIO、系统时钟、PWM、I2C、UART、ADC。

业务逻辑不在这里。它是底座。

## system

### system/board.h

板级初始化接口声明。

### system/board.c

板级初始化总管。分阶段初始化电源、GPIO、OLED、时钟、PWM、UART、ADC 和各硬件模块，并用 OLED 显示启动进度。

### system/delay.h

毫秒延时函数声明。

### system/delay.c

用 `CPUCLK_FREQ` 和 `delay_cycles()` 实现阻塞式毫秒延时。

### system/interrupt.h

中断相关声明，目前内容很少。

### system/interrupt.c

中断分发：

```text
GPIOA -> Encoder
GPIOB -> Key
UART0 -> JY61P
```

## hardware

### hardware/motor.h

定义 `MotorId`、`MotorDir` 和电机控制接口。

### hardware/motor.c

控制 TB6612。负责方向 GPIO、PWM 占空比、左右轮有符号速度命令、停车。

### hardware/key.h

定义按键 ID 和按键事件。

### hardware/key.c

PB9/PB8 按键模块。GPIOB 中断里记录 `g_keyEvent`，主循环用 `Key_PopEvent()` 取走。

### hardware/encoder.h

编码器接口声明。

### hardware/encoder.c

AB 相编码器计数。GPIOA 中断里根据 A/B 相状态更新左右轮累计计数。

### hardware/link.h

Exchange/调试串口接口声明。

### hardware/link.c

UART3 发送字节、字节数组、字符串。菜单监视和状态机会用它输出调试信息。

### hardware/gray.h

灰度模块接口。定义 7 路灰度编号，以及采样、读取、误差、阈值、校准函数。

### hardware/gray.c

灰度核心模块。用 ADC0/ADC1 的多个 MEM 槽读取 7 路传感器，做平均滤波、黑白阈值、防抖、mask、循迹误差和校准。

### hardware/oled.h

OLED 显示接口声明，包含画点、画线、显示字符、显示字符串、刷新、清屏、初始化等。

### hardware/oled.c

I2C OLED 驱动。通过 `OLED_WR_Byte()` 发送命令/数据，维护 `OLED_GRAM` 显存，刷新 128x64 屏幕。

### hardware/oledfont.h

OLED 点阵字库。主要是 ASCII 和部分中文点阵数组，不需要逐行读。

### hardware/jy61p.h

JY61P 姿态模块接口声明。

### hardware/jy61p.c

UART0 接收 JY61P 11 字节数据帧，校验后解析 yaw 角，提供 `JY61P_GetYawDeg()` 给路线模块使用。

### hardware/jq8400.h

JQ8400 语音模块接口声明。

### hardware/jq8400.c

UART1 基础发送模块。当前还只是发送字节/数组，完整播放协议可后续扩展。

## app

### app/main.c

C 程序入口。正常应调用 `Board_Init()`、`App_Init()`，然后在死循环里调用 `App_Task()`。

### app/app.h

应用层初始化和周期任务接口。

### app/app.c

应用总调度。初始化 tracking、route、speed control、motor test、menu、state machine。主循环里处理按键、状态机、速度闭环、菜单刷新、串口任务。

### app/menu.h

菜单模块接口。提供初始化、下一项、确认、周期刷新函数。

### app/menu.c

OLED 菜单和监视页面。负责主菜单、测试菜单、灰度校准提示、循迹状态、PID 数据、灰度数据、编码器数据等显示，也会周期性串口打印监视数据。

### app/state_machine.h

定义整车状态 `CarState` 和事件 `CarEvent`。

### app/state_machine.c

顶层状态机。根据事件切换菜单、校准、循迹、测试、监视、停止、错误等状态，并执行每个状态的入口动作和周期任务。

### app/motion.h

运动封装接口。

### app/motion.c

上层运动接口。根据 `CAR_ENABLE_SPEED_CONTROL` 决定命令走速度闭环还是直接给电机。

### app/tracking.h

循迹模块接口。

### app/tracking.c

循迹主任务。采灰度、算误差、调用异常处理器，最后给左右轮速度命令。

### app/tracking_exception.h

定义循迹异常状态和异常处理动作。

### app/tracking_exception.c

处理丢线、搜线、ADC 失败、宽线等情况。决定继续跑还是停车，并生成左右轮命令。

### app/route.h

路线外环接口和路线阶段枚举。

### app/route.c

路线外环。用编码器和 JY61P yaw 判断直道、接近拐角、转弯、出弯，并平滑调整基础速度和转向限幅。

### app/speed_control.h

速度闭环接口。

### app/speed_control.c

编码器 PI 速度闭环。把上层左右轮目标命令转换成电机 PWM 输出，并保存实际 tick、输出 PWM 给监视页面显示。

### app/motor_test.h

电机方向测试接口。

### app/motor_test.c

菜单里的电机方向确认流程。依次测试左正、左反、右正、右反、双正、双反，每步短时间自动停车。

## MDK-ARM

### MDK-ARM/light-car1.0.uvprojx

Keil 工程文件。记录参与编译的源文件、include 路径、编译选项、下载配置等。一般不手改，除非新增 `.c` 文件或调整工程配置。

### MDK-ARM/startup_mspm0g350x_uvision.s

启动汇编。定义栈、堆、中断向量表、复位入口和默认中断处理。复位后最终进入 C 运行环境，再到 `main()`。

### MDK-ARM/learning-notes

学习笔记目录。这里放灰度 ADC、OLED I2C、项目总览、源码阅读讲解，不挤进 README。

