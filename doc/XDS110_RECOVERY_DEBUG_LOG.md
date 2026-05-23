# XDS110 解锁与安全下载记录

日期：2026-05-23

本文记录 MSPM0G3507 地猛星最小系统板从“下载后无法再次连接”恢复到可下载、可运行的完整过程。后续遇到类似现象时，先按本文的低风险步骤排查，不要一上来擦 NONMAIN。

## 现象

- J-Link/CCS 下载后，后续再次下载失败。
- CCS / DSLite 可以识别 XDS110，但连接 `CORTEX_M0P` 失败。
- 典型失败信息：

```text
CORTEX_M0P: Error connecting to the target:
Connection to MSPM0 core failed.
Possible root causes:
1) Debug access within NONMAIN was disabled or enabled with password.
2) Peripheral mis-configuration (e.g improper watchdog or clock).
```

## 当时确认过的硬件与工具状态

- XDS110 已升级到固件 `3.0.0.41`。
- Windows 枚举到：
  - `COM17`：XDS110 Class Application/User UART
  - `COM18`：XDS110 Class Auxiliary Data Port
- CCS 后台进程已关闭，没有 `ccstudio.exe` 占用调试器。
- 用户已确认恢复安全版下载后 PA14 慢闪，说明 MAIN 程序已经运行。

## 工程侧安全措施

当前 `config/board_config.h` 临时启用：

```c
#define CAR_RECOVERY_SAFE_BUILD       (1U)
```

安全模式只做这些事：

- 使用内部 `SYSOSC`，关闭 HFXT / SYSPLL。
- 只给 GPIOA/GPIOB 和三路 UART 上电。
- PA14 慢闪，作为主循环存活信号。
- 三路 UART 周期打印：

```text
RECOVERY SAFE BUILD RUNNING, PA14 BLINK, UART OK
```

安全模式不会初始化 OLED/I2C、ADC、步进电机和 App 层，因此适合刚恢复芯片后的第一次下载。

## 已执行的恢复过程

1. 构建安全版固件：

```powershell
& "D:\Ti\light-car1.0ccs\tools\build_ccs.ps1" -Clean
```

构建成功输出：

```text
Build OK: D:\Ti\light-car1.0ccs\Debug\codex-build\light-car1.1ccs.out
```

2. 普通下载失败，说明问题还在芯片调试连接层：

```powershell
& "D:\Ti\light-car1.0ccs\tools\flash_xds110_safe.ps1" -SkipBuild
```

失败点是 `CORTEX_M0P` 无法连接。

3. 读取 Boot Diagnostic：

```powershell
& "D:\Ti\ccs\ccs_base\scripting\bin\dss.bat" "D:\Ti\light-car1.0ccs\tools\read_boot_diag.js"
```

读到：

```text
Device diagnostic read = 0x00000007
```

TI GEL 对 `0x00000007` 的解释是：可能存在嵌套异常/双 HardFault，或调试访问被禁用。TI 给出的恢复方法是 DSSM Factory Reset。

4. 用户明确授权后，执行 DSSM Factory Reset。

以后不要直接运行 JS，使用带确认参数的 wrapper：

```powershell
& "D:\Ti\light-car1.0ccs\tools\factory_reset_xds110.ps1" -ConfirmFactoryReset
```

本次关键成功日志：

```text
Initiating Device Factory Reset
Command Sent
Start hardware Reset using NRST
Command execution completed.
```

5. Factory Reset 后再次读取诊断：

```text
Device diagnostic read = 0x0004110A
```

状态已不再是 `0x00000007`。

6. 再次下载安全版：

```powershell
& "D:\Ti\light-car1.0ccs\tools\flash_xds110_safe.ps1" -SkipBuild
```

下载成功：

```text
Loading Program: D:\Ti\light-car1.0ccs\Debug\codex-build\light-car1.1ccs.out
Running...
Success
```

7. 用户实测现象：

- PA14 已经闪烁。
- 这证明芯片已恢复到可下载、可运行状态。

## 下次遇到同类问题的顺序

1. 先完全关闭 CCS / Debug Session。
2. 检查 XDS110：

```powershell
& "D:\Ti\ccs\ccs_base\common\uscif\xds110\xdsdfu.exe" -e
```

3. 构建安全版：

```powershell
& "D:\Ti\light-car1.0ccs\tools\build_ccs.ps1" -Clean
```

4. 先尝试普通安全下载：

```powershell
& "D:\Ti\light-car1.0ccs\tools\flash_xds110_safe.ps1" -SkipBuild
```

5. 如果连接核心失败，先读诊断：

```powershell
& "D:\Ti\ccs\ccs_base\scripting\bin\dss.bat" "D:\Ti\light-car1.0ccs\tools\read_boot_diag.js"
```

6. 只有在用户明确授权时，才执行 Factory Reset：

```powershell
& "D:\Ti\light-car1.0ccs\tools\factory_reset_xds110.ps1" -ConfirmFactoryReset
```

7. Factory Reset 后再下载安全版，观察 PA14 是否慢闪。

## 注意事项

- `flash_xds110_safe.ps1` 只下载 MAIN 程序，不执行 Factory Reset，不写 NONMAIN。
- `factory_reset_xds110.ps1` 会触发 DSSM Factory Reset，会重置 NONMAIN，必须谨慎。
- 串口监听 COM17/COM18 没收到文本不代表程序没跑；XDS110 虚拟串口未必接到当前固件打印的 TX 引脚。
- 当前安全版 UART 打印引脚是：
  - UART0：PA28 TX / PA31 RX
  - UART1：PB6 TX / PB7 RX
  - UART3：PB2 TX / PB3 RX
- 最可靠的恢复成功标志是 PA14 慢闪，以及 XDS110 能再次下载 MAIN 程序。
