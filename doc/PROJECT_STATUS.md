# light-car1.1ccs 工程状态

本文记录 CCS 版 `1.1ccs` 的当前结构、启动现象和硬件安全策略。

## 目录分层

- `app/`：应用层逻辑，包括菜单、状态机、循迹、速度命令、任务框架和步进测试入口。
- `hardware/`：硬件驱动，包括 OLED、按键、四步进电机、灰度、JY61P、JQ8400、Link。
- `system/`：板级初始化、延时和中断入口。
- `config/`：工程参数和引脚映射。
- `generated/`：CCS/SysConfig 风格生成层。
- `targetConfigs/`：CCS/J-Link/BSL 目标配置。
- `tools/`：命令行构建脚本。

## 当前主流程

`app/main.c` 保持干净：

当前为了芯片恢复，`CAR_RECOVERY_SAFE_BUILD = 1`。实际编译出来的固件会跳过 App 层，只执行 `Board_Init()` 和 `Board_Task()` 里的 PA14 慢闪与三路 UART 心跳恢复逻辑。

恢复正常小车固件前，需要在 `config/board_config.h` 把：

```c
#define CAR_RECOVERY_SAFE_BUILD       (1U)
```

改回：

```c
#define CAR_RECOVERY_SAFE_BUILD       (0U)
```

安全模式关闭后，主流程为：

1. `Board_Init()`：初始化电源、GPIO、OLED、时钟、步进 GPIO、UART、ADC 和硬件模块。
2. 无致命错误时执行 `App_Init()`。
3. 主循环持续执行 `Board_Task()`，无致命错误时执行 `App_Task()`。

## 开机 OLED 探针

OLED 上电会显示启动阶段，用来定位初始化卡点：

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

旧 TB6612 PWM 方案已替换为四个闭环步进驱动器 `STEP/DIR`：

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

PA12/PA13/PA22 已用于步进电机，不再接旧编码器。

## PA14 状态灯

`CAR_ENABLE_PA14_DEBUG_LED = 1`，PA14 当前可直接观察系统状态：

- 慢闪：主循环还活着。
- 快闪：存在非致命错误，例如 OLED/I2C 超时。
- 常亮：存在致命错误，例如时钟初始化失败。

2026-05-23 已通过 XDS110 恢复下载验证：`CAR_RECOVERY_SAFE_BUILD = 1` 时，安全版 MAIN 程序下载成功后 PA14 已实测慢闪。

## XDS110 恢复记录

本次芯片失联时，Boot Diagnostic 曾读到 `0x00000007`。经用户明确授权后执行 DSSM Factory Reset，随后安全版 MAIN 程序下载成功。

完整过程、命令和注意事项见：

- `doc/XDS110_RECOVERY_DEBUG_LOG.md`

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
- JQ8400 和 Link/Exchange 框架保留，但业务还没展开。
- `generated/ti_msp_dl_config.*` 是手工维护的 SysConfig 风格文件，再生成 SysConfig 时必须重新核对引脚。
