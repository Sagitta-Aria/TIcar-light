# 云台 STEP 与灰度快采样定时器

TIMG0 是 50 kHz 周期中断，周期 20 us。底盘已改为 TIMA0 PWM 编码电机，不再使用 TIMG0 生成底盘 STEP。

## 中断链路

```text
TIMG0_IRQHandler
  -> StepperPulse_HandleTimerInterrupt   两路云台STEP调度
  -> MotorNoYaw_TimerSample             内部分频到100 us读取数字灰度
```

| 文件 | 职责 |
| --- | --- |
| `generated/ti_msp_dl_config.c/h` | 配置 TIMG0 50 kHz 周期中断 |
| `system/interrupt.c` | TIMG0 ISR 入口 |
| `hardware/stepper_pulse.c/h` | PA7/PA8 两路云台 STEP 脉冲和斜坡 |
| `app/motor_no_yaw.c` | 数字灰度事件快采样 |

当前系统时钟和 BUSCLK 都是内部 SYSOSC 32 MHz，TIMG0 `LOAD = 639`。云台命令仍使用 step/s；底盘目标使用编码器 count/s，两者不可混用。

## 中断约束

- ISR 内禁止日志、OLED、I2C、阻塞 UART 和 FreeRTOS 非 ISR API。
- `MotorNoYaw_TimerSample()` 只读 GPIO、更新短计数器和置请求标志。
- `StepperPulse_HandleTimerInterrupt()` 只轮询两路云台电机。
- 修改时钟树后必须同步重新计算 TIMG0 load、TIMA0 PWM 周期和 FreeRTOS `configCPU_CLOCK_HZ`。
