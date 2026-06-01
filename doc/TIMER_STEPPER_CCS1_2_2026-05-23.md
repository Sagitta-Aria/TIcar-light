# ccs1.2 STEP 定时器调度说明

最后更新：2026-05-23

## 为什么改

上一版中 STEP 脉冲由 `Motor_Task()` 在主循环里补发。OLED 刷新、串口日志、按键菜单或灰度读取变慢时，STEP 节奏也会跟着抖。ccs1.2 把脉冲生成迁到 TIMG0 周期中断，主循环只负责设置目标速度和方向。

## 当前结构

| 文件 | 职责 |
| --- | --- |
| `hardware/motor.c/h` | 电机逻辑接口：设置方向、速度命令、停车、读取当前命令 |
| `hardware/stepper_pulse.c/h` | STEP 脉冲调度：把速度命令换算成 Hz，在 TIMG0 ISR 中翻转 STEP |
| `generated/ti_msp_dl_config.c/h` | 配置 TIMG0 为 50us 周期定时器 |
| `system/interrupt.c` | `TIMG0_IRQHandler()` 分发到 `StepperPulse_HandleTimerInterrupt()` |

## 定时器参数

- 定时器：`TIMG0`
- 周期：50us
- 中断频率：20kHz
- STEP 高电平：1 个 tick，约 50us
- 当前换算：`4000` 命令约等于 `400 step/s`

这组参数偏保守，目的是先保证实车接线、方向和闭环步进驱动器响应稳定。后续确认驱动器脉冲输入能力后，可以逐步提高命令到 Hz 的换算比例。

## 硬件现象

- STEP 输出仍由 TIMG0 调度；当前云台左右轴使用 PA7，云台上下轴使用 PA8，底盘临时使用 PA12/PA13。
- 低速测试时，电机速度和上一版接近，但声音应该更均匀。
- OLED 刷新或串口日志变多时，电机 STEP 不会再被主循环明显拖慢。
- 如果 OLED/I2C 异常，程序仍会走原来的超时和错误兜底，STEP 模块本身没有等待和死循环。

## 注意事项

- TIMG0 ISR 里禁止打印日志、刷 OLED、做 I2C/UART 阻塞等待。
- `Motor_Task()` 现在保留为空任务，方便以后加入温度、告警或驱动器反馈处理。
- 如果将来要极高速同步 STEP，可以再评估硬件 PWM/CCP 或多定时器方案；那一步才需要重新确认引脚复用和定时器通道。
