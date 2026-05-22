# 1.1ccs 代码风格说明

这份文档给后续读代码和继续开发用，重点是保持工程可查、可测、遇到硬件异常不会死锁。

## 分层风格

- `main.c` 只做工程入口，不放外设细节。
- `system/board.c` 负责板级初始化顺序、启动 OLED 探针和板级错误。
- `hardware/xxx.c` 只负责一个硬件模块，例如 `motor.c` 只管 STEP/DIR，`oled.c` 只管 SSD1306/I2C。
- `app/xxx.c` 负责业务状态，不直接写寄存器。
- `config/board_config.h` 放可调参数，`config/pin_map.h` 放逻辑命名和生成层宏的桥接。

## 命名风格

- 模块函数用 `Module_Action()`，例如 `Motor_Set()`、`OLED_TryRecover()`、`Board_ReportError()`。
- 内部静态函数用同一模块前缀，例如 `Motor_TaskOne()`。
- 全局静态变量用 `g_` 前缀，例如 `g_boardErrors`。
- 宏按模块分组并全大写，例如 `CAR_STEPPER_MAX_COMMAND`。
- 枚举值带模块语义，例如 `MOTOR_CHASSIS_LEFT`、`BOARD_ERROR_CLOCK`。

## 硬件等待规则

- 不写永久等待外设状态的 `while`。
- I2C、UART、ADC、PLL、GPIO 中断清理都必须有超时或服务次数上限。
- 出错后优先“记录错误并退出”，不要在底层无限重试。
- 能恢复的错误由模块提供显式恢复函数，例如 `OLED_TryRecover()`。

## 注释风格

- 新函数前优先写中文短注释，说明“作用、使用场景、限制”。
- 不解释显而易见的赋值。
- 对引脚冲突、下载风险、硬件副作用要明确写注释。
- 对兼容旧接口的代码要写清楚，例如 `Motor_SetSpeed()` 只是兼容左右轮命令。

## 1.1ccs 的电机框架

- 上层仍然可以用旧的左右轮命令接口。
- 底层已经换成四个 `STEP/DIR` 闭环步进驱动器。
- `Motor_Task()` 是当前的软件步进调度点。
- 方向反相、加速度曲线、高速同步和驱动器反馈闭环，后续应在 `hardware/motor.c` 或新建 `app/motion.c` 中模块化实现。

## 生成层维护

- `generated/ti_msp_dl_config.*` 现在是 CCS/SysConfig 风格手工维护文件。
- 改引脚必须同步更新 `doc/PIN_ASSIGNMENT.md`、`config/pin_map.h` 和生成层宏。
- 再次用 SysConfig 图形界面生成后，必须重点复查 PA0/PA1、PA14、PA19/PA20、PA21/PA23、PB14-PB17。

## 验证习惯

每次较大改动至少跑：

```powershell
& "D:\Ti\light-car1.0ccs\tools\build_ccs.ps1" -Clean
```

硬件下载前先确认：

- PA19/PA20 没接外设。
- PA0/PA1 有上拉，OLED 没短路。
- PA14 只接 LED。
- 步进驱动器逻辑电平和 MCU 共地。
- 电机电源不从 MCU 取。
