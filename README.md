# light-car1.1ccs

TI CCS / TI Arm Clang version of the MSPM0G3507 laser tracking car firmware.

当前仓库路径仍是：

```powershell
D:\Ti\light-car1.0ccs
```

但 GitHub 版本线已经升级为 `1.1ccs`。这一版面向地猛星 MSPM0G3507 最小系统板，重点改动是把旧 TB6612 直流电机方案切到四个闭环步进驱动器的 `STEP/DIR` 方案。

## 当前结构

- `app/`：菜单、状态机、循迹、速度命令、步进电机测试入口。
- `hardware/`：OLED、按键、四步进电机、灰度 ADC、UART 模块驱动。
- `system/`：板级初始化、延时、中断入口和错误兜底。
- `config/`：工程参数和引脚映射。
- `generated/`：CCS/SysConfig 风格生成层。
- `targetConfigs/`：J-Link / BSL 目标配置。
- `tools/`：命令行构建和下载辅助脚本。
- `doc/`：接线表、状态说明和代码风格。

## 构建

```powershell
& "D:\Ti\light-car1.0ccs\tools\build_ccs.ps1" -Clean
```

构建只编译链接，不下载、不擦除芯片。成功输出：

```text
D:\Ti\light-car1.0ccs\Debug\codex-build\light-car1.1ccs.out
```

## 当前临时安全模式

为了 XDS110 解锁后的第一次恢复下载，当前 `config/board_config.h` 中：

```c
#define CAR_RECOVERY_SAFE_BUILD       (1U)
```

此模式下主工程只初始化 PA14 和三路 UART 心跳，不进入 `App_Init/App_Task`，也不初始化 OLED/I2C/PLL/ADC/步进电机。串口默认 115200，会周期打印 `RECOVERY SAFE BUILD RUNNING, PA14 BLINK, UART OK`。确认 PA14 稳定闪烁、串口有输出、芯片可以重复下载后，再把它改回 `0U` 恢复完整小车固件。

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
- PA19/PA20 是 SWD 下载脚，工程不复用。
- PB14/PB15/PB16/PB17 是板载 SPI Flash，工程不复用。

详细接线和当前状态见：

- `doc/PIN_ASSIGNMENT.md`
- `doc/PROJECT_STATUS.md`
- `doc/CODE_STYLE_1_1CCS.md`
- `doc/XDS110_RECOVERY_DEBUG_LOG.md`
