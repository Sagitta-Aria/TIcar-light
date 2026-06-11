# light-car CCS

MSPM0G3507 laser tracking car firmware for TI CCS / TI Arm Clang.

当前工作工程：

```powershell
D:\Ti\light-car1.0ccs
```

当前开发分支：`1.2ccsadc`

固件版本线：`ccs1.2`

最后整理：2026-05-24

Keil 旧工程在 `D:\激光循迹\light-car1.0`，不要和本 CCS 工程混用。

## 当前目标

这一版面向地猛星 MSPM0G3507 最小系统板，电机控制采用四个闭环步进驱动器的 `STEP/DIR` 方案。

当前重点：

- 底盘左右步进电机用于循迹和任务运动。
- 云台两个步进电机用于激光方向控制。
- 视觉模块通过 UART3/Link 提供激光点或目标点坐标。
- 墙面画圆优先走视觉闭环：摄像头识别墙面/激光点，MCU 根据视觉误差驱动云台。
- 底盘位姿解算已保留，用于后续一边走一边补偿墙面目标，但当前不强依赖它。

## 工程结构

- `app/`：应用层逻辑，包含菜单、状态机、循迹、路线、云台控制、云台测试、车体位姿解算。
- `hardware/`：硬件驱动，包含 OLED、按键、四步进电机、STEP 定时器调度、灰度输入、JY61P、Link、日志串口。
- `system/`：板级初始化、延时、中断入口、错误状态。
- `config/`：工程参数和引脚映射。
- `generated/`：CCS/SysConfig 风格生成代码。
- `targetConfigs/`：J-Link / XDS110 / BSL 目标配置。
- `tools/`：构建、下载、恢复辅助脚本。
- `doc/`：接线表、状态说明、恢复记录和代码风格。

## 主流程

`app/main.c` 保持干净，只做板级初始化、应用初始化和主循环调度：

```c
Board_Init();
if (Board_HasFatalError() == 0U) {
    App_Init();
}

while (1) {
    Board_Task();
    if (Board_HasFatalError() == 0U) {
        App_Task();
    }
}
```

## 菜单

OLED 当前主菜单：

```text
Gimbal
Motor
Mission
```

`Gimbal` 二级菜单：

```text
Near Center
Near Circle
Mid Center
Mid Circle
Far Center
Far Circle
```

`Motor` 二级菜单：

```text
Step Test
Track Run
```

灰度校准页：

```text
Cal Done: YES/NO
Save Exit
No Save Exit
```

说明：

- `Gimbal`：选择六套 `staticconfig` 云台参数之一，K2 进入视觉追踪。
- `Motor / Step Test`：底盘固定平均 STEP 测试，到目标脉冲后自动停车。
- `Motor / Track Run`：读取灰度和 JY61P yaw 的真实循迹状态机。
- `Mission`：任务入口，具体任务流程后续继续补。

## 主要模块

### 电机与 STEP 调度

- `hardware/motor.c/h`
  - 负责四个步进电机的速度命令、DIR 方向、停车和命令读取。
  - 逻辑电机编号：
    - `MOTOR_CHASSIS_LEFT`
    - `MOTOR_CHASSIS_RIGHT`
    - `MOTOR_GIMBAL_1`
    - `MOTOR_GIMBAL_2`

- `hardware/stepper_pulse.c/h`
  - 使用 TIMG0 每 20us 中断调度 STEP 脉冲。
  - 50kHz tick。
  - 支持读取各电机累计 STEP 输出计数。
  - 速度命令直接使用 SPS，并带 1ms 斜坡限速。

STEP 接线：

```text
底盘左：PA12 STEP，PA22 DIR
底盘右：PA13 STEP，PB24 DIR
云台左右轴：PA7 STEP，PB18 DIR
云台上下轴：PA8 STEP，PA9 DIR
```

### 云台控制

- `app/gimbal.c/h`
  - 二维云台闭环控制。
  - 视觉差值入口：

```c
Gimbal_UpdateFromCameraError(targetMinusCurrentX, targetMinusCurrentY);
```

控制逻辑：

```text
errorX = targetMinusCurrentX + offsetX
errorY = targetMinusCurrentY + offsetY
误差进入死区：停止对应轴
误差超过死区：按比例输出 STEP 命令
```

deadband、kp、kd、min/maxSpeed、offset 都来自 `app/staticconfig.c` 当前 active 参数。

当前默认映射：

```text
X 轴/左右 -> MOTOR_GIMBAL_1 -> PA7 STEP / PB18 DIR / PA31 EN
Y 轴/上下 -> MOTOR_GIMBAL_2 -> PA8 STEP / PA9 DIR / PB19 EN
```

方向反了优先改：

```c
CAR_GIMBAL_X_REVERSE
CAR_GIMBAL_Y_REVERSE
```

### 云台测试

- `app/gimbal_test.c/h`
  - 菜单选择 `Gimbal` 六套参数之一后启用。
  - 清空 Link 接收缓存。
  - 接收视觉差值并调用 `Gimbal_UpdateFromCameraError()`。

视觉输入行尾用 `\n` 或 `\r`：

```text
centerDx,centerDy;circleDx,circleDy
```

dx/dy 已经是 `target - current`；MCU 解析后放大到 0.1 像素单位。当前 active 参数的 `useCircleError=0` 使用前两个数，`useCircleError=1` 使用后两个数。

实车现象：

- 进入 `Gimbal Test` 后底盘停车。
- 未收到视觉数据：OLED 显示 `Waiting Link`，云台不动。
- 收到有效坐标：OLED 显示 `Tracking`，云台按视觉误差追踪。
- 连续约 200ms 没有视觉更新，云台自动停止。

### 云台电机测试

- `app/gimbal_motor_test.c/h`
  - 当前是云台 yaw/pitch 持续 SPS 输出测试。
  - 进入后同时输出 `MOTOR_GIMBAL_1` 和 `MOTOR_GIMBAL_2`。
  - K1/K2 可按 `CAR_STEPPER_SPEED_STEP_SPS` 调整测试速度。
  - OLED 显示 yaw/pitch 当前 SPS。

关键参数：

```c
CAR_GIMBAL_TEST_YAW_SPEED_SPS
CAR_GIMBAL_TEST_PITCH_SPEED_SPS
CAR_GIMBAL_MOTOR_TEST_REVERSE
CAR_STEPPER_SPEED_STEP_SPS
```

实车现象：

- `Speed`：只验证云台两个轴能按 STEP/DIR 持续输出。
- 长按 K2 退出后停止云台轴，并恢复默认 EN 状态。

### 四电机使能测试

- `hardware/motor_enable.c/h`
  - 管四路步进驱动器 EN 输出。
  - 默认按 ZDT 示例配置为低电平使能。
- `app/motor_enable_test.c/h`
  - 菜单进入 `Gimbal Test / Enable Test` 后启用。
  - 只拉四路 EN，不输出 STEP 脉冲。

接线：

```text
底盘左 EN  -> PA2
底盘右 EN  -> PA28
云台左右 EN -> PA31
云台上下 EN -> PB19
```

实车现象：进入 `Enable Test` 后，如果驱动器供电、COM、GND、EN 极性都正确，四个电机会立刻抱住；退出后会释放。

### Link / 视觉串口

- `hardware/link.c/h`
  - 使用 UART3。
  - PB2 TX，PB3 RX。
  - 115200。
  - 中断里只收字节并拼行，不做业务解析、不打印日志。
  - 主循环通过 `Link_PopLine()` 取完整行。

当前 Link 主要给云台测试使用，后续可扩展为视觉协议层。

### 车体位姿解算

- `app/pose_solver.c/h`
  - 用底盘左右 STEP 输出计数估算位移。
  - 用 JY61P yaw 作为车体朝向。
  - 输出车体相对零点的 `xMm/yMm/travelMm/yawDeg`。

坐标约定：

```text
yMm：yaw=0 时车头前进方向
xMm：车体右侧方向
yawDeg：相对启动或 PoseSolver_Reset() 时的航向角
```

当前 STEP 到毫米比例只是占位：

```c
CAR_POSE_STEP_TO_MM_NUMERATOR
CAR_POSE_STEP_TO_MM_DENOMINATOR
```

默认 `4 step = 1 mm`，后续必须用尺子实车标定。

注意：

- 这个模块只解算车体位姿，不解算云台绝对姿态。
- 云台当前没有独立编码器或回零开关，不能可靠知道绝对角度。
- 墙面画圆当前推荐视觉闭环，不优先做复杂三维几何模型。

### JY61P

- `hardware/jy61p.c/h`
  - UART1，PB6 TX / PB7 RX。
  - 解析 JY61P 角度帧。
  - 当前缓存 roll/pitch/yaw。
  - `PoseSolver` 主要使用 yaw。

### 灰度输入与循迹

- `hardware/gray.c/h`
  - 当前默认数字灰度输入模式。
  - 传感器模块自己完成黑白比较，MCU 读取 GPIO 高低电平。

- `app/tracking.c/h`
  - 根据灰度数字量计算循迹误差。
  - 输出底盘左右 STEP 命令。

- `app/tracking_exception.c/h`
  - 处理丢线、搜线、传感器异常等情况。

### 路线

- `app/route.c/h`
  - 使用底盘 STEP 输出计数估算路线距离。
  - 不再使用旧编码器模块。
  - 当前仍是任务路线框架，后续按实车继续标定距离和状态机。

## UART 分配

```text
UART0：PA10 TX / PA11 RX，Type-C CH340 日志，115200
UART1：PB6  TX / PB7  RX，JY61P，115200
UART3：PB2  TX / PB3  RX，视觉/Exchange/Link，115200
```

JQ8400 语音模块当前暂停接入，不占串口。

## 当前保留和禁止复用引脚

- PA0 / PA1：OLED I2C0，开漏释放，必须上拉。
- PA5 / PA6：外部晶振硬件保留，当前软件默认不用 PLL。
- PA10 / PA11：Type-C CH340 日志 UART0，同时也是 BSL 数据线。
- PA18：BSL invoke，保留恢复入口。
- PA14：状态 LED。
- PA19 / PA20：SWD 下载调试脚，禁止复用。
- PA21 / PA23：VREF 相关，暂不做普通 GPIO/ADC。
- PB14 / PB15 / PB16 / PB17：板载 SPI Flash，禁止应用层复用。

## 状态 LED

PA14：

```text
慢闪：主循环存活
快闪：非致命错误，例如 OLED/I2C 超时
常亮：致命错误，例如时钟失败
```

## 构建

```powershell
& "D:\Ti\light-car1.0ccs\tools\build_ccs.ps1" -Clean
```

构建只编译链接，不下载、不擦除芯片。成功输出类似：

```text
Build OK: D:\Ti\light-car1.0ccs\Debug\codex-build\light-car-ccs1.2.out
```

## 下载和恢复

默认不要随便下载、擦除或 Factory Reset。

普通 XDS110 安全下载脚本：

```powershell
& "D:\Ti\light-car1.0ccs\tools\flash_xds110_safe.ps1" -SkipBuild
```

Factory Reset 必须显式确认：

```powershell
& "D:\Ti\light-car1.0ccs\tools\factory_reset_xds110.ps1" -ConfirmFactoryReset
```

恢复安全模式开关：

```c
#define CAR_RECOVERY_SAFE_BUILD       (0U)
```

只有救板子或首次恢复下载时才临时改成 `1U`。此模式只初始化 PA14 和三路 UART 心跳，不进入 App，不初始化 OLED/I2C/PLL/灰度/步进电机。

## 安全策略

- 正常构建使用内部 `SYSOSC 32MHz`，不启用 HFXT/SYSPLL。
- OLED/I2C 所有等待都有超时。
- I2C 异常时执行 bus clear，不允许死等。
- 中断里不打印日志、不刷 OLED、不做 I2C/UART 阻塞等待。
- TIMG0 只负责 STEP 调度。
- UART3 中断只收字节入缓存，业务解析放主循环。
- 不写 NONMAIN，不改 BSL 配置，不做 mass erase。

## GitHub 分支

当前推送分支：`1.2ccsadc`。

如果 GitHub 首页仍显示旧代码，需要在 GitHub 仓库设置里把默认分支改到当前需要展示的 CCS 分支。

## 后续计划

- 实车测试 `Gimbal Test` 的视觉追踪方向、死区和增益。
- 确认视觉模块实际输出协议，必要时把 Link 从“行解析”升级为正式帧协议。
- 做墙面画圆任务框架：视觉给圆心/当前激光点，MCU 生成圆周目标点并驱动云台追踪。
- 标定底盘 STEP 到毫米比例。
- 如后续需要云台开环角度，再增加云台回零、step/deg 标定和 `GimbalPose`。
