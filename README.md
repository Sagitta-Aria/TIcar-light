# light-car1.1ccs

TI CCS / TI Arm Clang version of the MSPM0G3507 laser tracking car firmware.

当前最新开发分支：`1.1ccs`
最后整理：2026-05-23

当前仓库路径仍是：

```powershell
D:\Ti\light-car1.0ccs
```

但 GitHub 版本线已经升级为 `1.1ccs`。这一版面向地猛星 MSPM0G3507 最小系统板，重点改动是把旧 TB6612 直流电机方案切到四个闭环步进驱动器的 `STEP/DIR` 方案。

## 当前结构

- `app/`：菜单、状态机、循迹、速度命令、步进电机测试入口。
- `hardware/`：OLED、按键、四步进电机、灰度 ADC、日志/JY61P/视觉 UART 模块驱动。
- `system/`：板级初始化、延时、中断入口和错误兜底。
- `config/`：工程参数和引脚映射。
- `generated/`：CCS/SysConfig 风格生成层。
- `targetConfigs/`：J-Link / BSL 目标配置。
- `tools/`：命令行构建和下载辅助脚本。
- `doc/`：接线表、状态说明和代码风格。

## 当前进度

- 主流程已恢复为完整小车固件：`CAR_RECOVERY_SAFE_BUILD = 0U`。
- OLED/I2C 已加超时、错误兜底和 bus clear，不会因为 OLED 线松死等。
- PA14 是状态灯：慢闪表示主循环存活，快闪表示非致命错误，常亮表示致命错误。
- Type-C CH340 日志走 `UART0 PA10/PA11`，正常 115200。
- JY61P 使用 `UART1 PB6/PB7`；视觉/Exchange 使用 `UART3 PB2/PB3`；JQ8400 语音模块暂停。
- 按键 PB9/PB8 已加软件消抖；OLED 菜单只用下半区，选中项固定在中间行。
- 四个闭环步进电机已切到 STEP/DIR 框架，适合低速接线和方向测试。

## 构建

```powershell
& "D:\Ti\light-car1.0ccs\tools\build_ccs.ps1" -Clean
```

构建只编译链接，不下载、不擦除芯片。成功输出：

```text
D:\Ti\light-car1.0ccs\Debug\codex-build\light-car1.1ccs.out
```

## 恢复安全模式开关

正常小车固件下，当前 `config/board_config.h` 中：

```c
#define CAR_RECOVERY_SAFE_BUILD       (0U)
```

如果要做救板子或首次恢复下载，再临时改成 `1U`。此模式下主工程只初始化 PA14 和三路 UART 心跳，不进入 `App_Init/App_Task`，也不初始化 OLED/I2C/PLL/ADC/步进电机。串口默认 115200，会周期打印 `RECOVERY SAFE BUILD RUNNING, PA14 BLINK, UART OK`。

## XDS110 安全下载

当前已验证：XDS110 执行 DSSM Factory Reset 后，安全版 MAIN 程序可下载成功，PA14 已实测慢闪。

普通安全下载命令：

```powershell
& "D:\Ti\light-car1.0ccs\tools\flash_xds110_safe.ps1" -SkipBuild
```

读取 Boot Diagnostic：

```powershell
& "D:\Ti\ccs\ccs_base\scripting\bin\dss.bat" "D:\Ti\light-car1.0ccs\tools\read_boot_diag.js"
```

Factory Reset 必须显式确认：

```powershell
& "D:\Ti\light-car1.0ccs\tools\factory_reset_xds110.ps1" -ConfirmFactoryReset
```

完整恢复记录见 `doc/XDS110_RECOVERY_DEBUG_LOG.md`。

## 下载建议

如果要用 J-Link，优先用下载后保持 halt 的脚本：

```powershell
JLink.exe -CommandFile "D:\Ti\light-car1.0ccs\tools\jlink_download_halt.jlink"
```

该脚本 `loadfile` 后会停住 CPU，不会下载完立刻跑飞固件。构建脚本本身不会触碰硬件。

## 1.1ccs 安全策略

- 默认使用内部 `SYSOSC 32MHz`，不启用外部 HFXT / SYSPLL。
- OLED I2C0 使用 PA0/PA1，软件等待都有超时。
- OLED 超时后会做 bus clear：临时切 GPIO 开漏、打 9 个 SCL 脉冲、生成 STOP，再恢复 I2C。
- PA14 固定作为状态灯，灰度 S1 已迁走。
- UART0 使用 PA10/PA11 走 Type-C CH340 日志，PA18 拉低时仍可进入 BSL。
- JY61P 使用 UART1 PB6/PB7；JQ8400 暂停接入，不初始化、不占串口。
- Link/Exchange 使用 UART3 PB2/PB3，留给视觉模块。
- PA19/PA20 是 SWD 下载脚，工程不复用。
- PB14/PB15/PB16/PB17 是板载 SPI Flash，工程不复用。

详细接线和当前状态见：

- `doc/PIN_ASSIGNMENT.md`
- `doc/PROJECT_STATUS.md`
- `doc/ROADMAP.md`
- `doc/CODE_STYLE_1_1CCS.md`
- `doc/XDS110_RECOVERY_DEBUG_LOG.md`

## GitHub 分支说明

`1.1ccs` 是当前最新 CCS 版本线；`keil1.0` 保留旧 Keil 工程；`main` 仍是旧默认入口时会显得版本很老。GitHub 仓库默认分支建议改成 `1.1ccs`。
