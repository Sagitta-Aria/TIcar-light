# 云台 STEP 与灰度快采样定时器

当前将两个高频来源拆开：TIMG6 是 20 kHz 云台 STEP 调度，TIMG0 是 10 kHz
数字灰度采样。底盘使用 TIMA0 硬件 PWM，不产生周期中断。

## 中断链路

```text
TIMG6_IRQHandler
  -> StepperPulse_HandleTimerInterrupt   两路云台STEP调度

TIMG0_IRQHandler
  -> MotorNoYaw_TimerSample             每100 us读取数字灰度
```

| 文件 | 职责 |
| --- | --- |
| `generated/ti_msp_dl_config.c/h` | 配置 TIMG6 20 kHz 和 TIMG0 10 kHz |
| `system/interrupt.c` | 两个独立 ISR 入口 |
| `hardware/stepper_pulse.c/h` | PA7/PA8 两路云台 STEP 脉冲和斜坡 |
| `app/motor_no_yaw.c` | 数字灰度事件快采样 |

当前系统时钟和 BUSCLK 都是内部 SYSOSC 32 MHz，TIMG6 `LOAD = 1599`，
TIMG0 `LOAD = 3199`。TIMG6 只在任一云台轴目标非零时运行，TIMG0 只在
Task1/Task4 循迹时运行。云台命令使用 step/s；底盘目标使用编码器
count/20ms，两者不可混用。

## 中断约束

- ISR 内禁止日志、OLED、I2C、阻塞 UART 和 FreeRTOS 非 ISR API。
- `MotorNoYaw_TimerSample()` 只读 GPIO、更新短计数器和置请求标志。
- `StepperPulse_HandleTimerInterrupt()` 只轮询两路云台电机。
- 修改时钟树后必须同步重新计算 TIMG6/TIMG0 load、TIMA0 PWM 周期和 FreeRTOS `configCPU_CLOCK_HZ`。
