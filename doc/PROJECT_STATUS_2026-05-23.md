# light-car1.1ccs 工程状态

最后更新：2026-05-23

本文记录 CCS 版 `1.1ccs` 的当前结构、启动现象和硬件安全策略。

## 目录分层

- `app/`：应用层逻辑，包括菜单、状态机、循迹、速度命令、任务框架和步进测试入口。
- `hardware/`：硬件驱动，包括 OLED、按键、四步进电机、灰度、Type-C 日志、JY61P、Link。JQ8400 框架保留但当前暂停接入。
- `system/`：板级初始化、延时和中断入口。
- `config/`：工程参数和引脚映射。
- `generated/`：CCS/SysConfig 风格生成层。
- `targetConfigs/`：CCS/J-Link/BSL 目标配置。
- `tools/`：命令行构建脚本。

## 当前主流程

`app/main.c` 保持干净：

当前正常小车固件使用 `CAR_RECOVERY_SAFE_BUILD = 0`。程序会执行完整 `Board_Init()`，无致命错误时进入 `App_Init()` 和 `App_Task()`。

只有救板子或首次恢复下载时，才需要在 `config/board_config.h` 临时改成：

```c
#define CAR_RECOVERY_SAFE_BUILD       (1U)
```

确认 PA14 稳定慢闪、可重复下载后，再改回：

```c
#define CAR_RECOVERY_SAFE_BUILD       (0U)
```

保持正常模式时，主流程为：

1. `Board_Init()`：初始化电源、GPIO、OLED、时钟、步进 GPIO、UART、ADC 和硬件模块。
2. 无致命错误时执行 `App_Init()`。
3. 主循环持续执行 `Board_Task()`，无致命错误时执行 `App_Task()`。

## 已完成并验证的基础项

- XDS110 MAIN 下载成功，不需要 Factory Reset，不写 NONMAIN。
- PA14 状态灯可用：慢闪为主循环存活，快闪为 OLED/I2C 等非致命错误，常亮为致命错误。
- OLED/I2C 初始化已加超时、错误兜底和 bus clear，线松或 OLED 未响应时不会死等。
- Type-C CH340 日志串口框架已接入 `UART0 PA10/PA11`，调试日志不再占用视觉串口；`CAR_ENABLE_LOG_UART` 是全局日志开关。
- PB9/PB8 按键已加约 40ms 软件消抖，菜单切换不再依赖临时 PA14 翻转调试。
- OLED 菜单和启动探针页避开顶部黄色区域；滚动菜单当前项固定在中间行。
- 四个闭环步进电机已改成 `STEP/DIR` 控制框架，主循环每轮有限步进输出，不长时间阻塞。

## 开机 OLED 探针

OLED 上电会在下半区显示启动阶段，用来定位初始化卡点；顶部黄色区域保持清空：

- `RUN Clock`
- `RUN Stepper`
- `RUN UART`
- `RUN Gray ADC`
- `RUN Motor`
- `RUN Gray`
- `RUN Key`
- `RUN App`

如果 OLED 没接好或 I2C 异常，OLED 驱动会超时退出，板级记录 `BOARD_ERROR_OLED_I2C`，程序不死等。

## I2C/OLED 保护

- PA0/PA1 使用 I2C0，软件配置为 Hi-Z 释放，配合上拉形成开漏总线。
- 内部弱上拉已打开，但实车仍建议 SDA/SCL 各接 4.7k~10k 到 3.3V。
- 所有 I2C 等待都有计数超时。
- 超时、NACK、仲裁丢失后会复位当前传输。
- 恢复时临时把 PA0/PA1 切成 GPIO 开漏，手动打 9 个 SCL 脉冲并生成 STOP，再切回 I2C0 重新初始化 OLED。

## 时钟与下载安全

为降低“下载后立即运行导致芯片失联”的风险，当前默认只用内部 `SYSOSC 32MHz`：

- `SYSCFG_DL_ENABLE_HFXT_PLL = 0`
- `CPUCLK_FREQ = 32000000`
- 不启用 PA5/PA6 外部晶振。
- 不切 SYSPLL。

如果以后恢复 HFXT/SYSPLL，也必须保留超时等待，不能用永久 `while` 等 PLL 锁定。

## 四步进电机状态

当前电机方案为四个闭环步进驱动器 `STEP/DIR`：

| 电机 | STEP | DIR |
| --- | --- | --- |
| 底盘左 | PA7 | PB18 |
| 底盘右 | PA8 | PA9 |
| 云台 1 | PA12 | PA22 |
| 云台 2 | PA13 | PB24 |

当前不接 EN。`Motor_Task()` 每轮最多给单个电机补发有限个 STEP 脉冲，避免长时间阻塞主循环。后续如果要高速度/高同步性，应把 STEP 产生迁到定时器中断或硬件定时器。

## 灰度与编码器

灰度 S1-S7 已改到 PA15/PA16/PA17/PA24/PA25/PA26/PA27，避开 PA14 LED、PA18 BSL、PA21/PA23 VREF。

外部编码器输入当前关闭：

- `CAR_ENABLE_ENCODER_INPUTS = 0`
- `CAR_ENABLE_SPEED_CONTROL = 0`

PA12/PA13/PA22 已用于步进电机，不再接原编码器接口。

## PA14 状态灯

`CAR_ENABLE_PA14_DEBUG_LED = 1`，PA14 当前可直接观察系统状态：

- 慢闪：主循环还活着。
- 快闪：存在非致命错误，例如 OLED/I2C 超时。
- 常亮：存在致命错误，例如时钟初始化失败。

2026-05-23 已通过 XDS110 恢复下载验证：`CAR_RECOVERY_SAFE_BUILD = 1` 时，安全版 MAIN 程序下载成功后 PA14 已实测慢闪。

## 当前 UART 分配

| 用途 | UART | 引脚 | 说明 |
| --- | --- | --- | --- |
| Type-C 日志 / BSL 数据线 | UART0 | PA10 TX / PA11 RX | 正常固件打印行为日志；PA18 拉低进 BSL 时同线复用下载 |
| JY61P 姿态模块 | UART1 | PB6 TX / PB7 RX | 语音模块暂停后释放给姿态模块 |
| Link/Exchange 视觉模块 | UART3 | PB2 TX / PB3 RX | 先保留视觉通信框架 |
| JQ8400 语音模块 | 暂停 | 不接 | 不初始化，不占用串口 |

日志关闭方式：

```c
#define CAR_ENABLE_LOG_UART            (0U)
```

关闭后 `LOG_*` 宏为空操作，启动、按键、状态机、路线阶段、循迹异常、电机测试等行为日志不再输出。PA10/PA11 仍作为 Type-C/BSL 保留脚，不建议复用给其它外设。

## XDS110 恢复记录

本次芯片失联时，Boot Diagnostic 曾读到 `0x00000007`。经用户明确授权后执行 DSSM Factory Reset，随后安全版 MAIN 程序下载成功。

完整过程、命令和注意事项见：

- `doc/XDS110_RECOVERY_DEBUG_LOG_2026-05-23.md`

## 中断处理

`system/interrupt.c` 已补 `GROUP1_IRQHandler`，GPIOA/GPIOB 不会落到启动文件默认死循环。按键、UART 接收和兜底 UART 清理都有限次服务上限，避免噪声把 CPU 困在 ISR。

## 构建

```powershell
& "D:\Ti\light-car1.0ccs\tools\build_ccs.ps1" -Clean
```

输出：

```text
D:\Ti\light-car1.0ccs\Debug\codex-build\light-car1.1ccs.out
```

构建不会下载、擦除或 mass erase。

## J-Link 下载建议

```powershell
JLink.exe -CommandFile "D:\Ti\light-car1.0ccs\tools\jlink_download_halt.jlink"
```

该脚本下载后保持 `halt`，便于确认程序没有立刻运行到异常状态。不要在不确认接线的情况下直接 reset/go。

## 已知限制

- 当前 STEP 由主循环软件调度，适合低速验证接线和方向，不适合最终高速同步控制。
- 步进驱动器反馈 UART/告警输入还没有接入业务闭环。
- JQ8400 语音模块暂停接入；Link/Exchange 框架保留，但视觉业务还没展开。
- JY61P 只完成 UART 帧解析和 yaw 缓存，姿态闭环还没有真正参与完整路线控制。
- 灰度循迹和路线状态机有框架，但实车参数、阈值、路口策略仍需要现场调试。
- `generated/ti_msp_dl_config.*` 是手工维护的 SysConfig 风格文件，再生成 SysConfig 时必须重新核对引脚。
