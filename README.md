# m0-light-rtos

MSPM0G3507 光电小车工程，使用 CCS Theia、TI Arm Clang 4.0.4 LTS 和 FreeRTOS V11.3.0。

工程目录：

```text
D:\Ti\m0-light-rtos
```

本工程以 `D:\Ti\light-car1.0ccs` 的比赛 Task1～4、菜单、视觉通信和云台控制为业务基础，移植 `Trace-f407` 已整理的 FreeRTOS 分层。原工程保持不变。

## 产品 Profile 与复用库

工程用显式编译期 Profile 区分两种产品：`Gmr` 是当前天猛星赛题控制台，
`Full` 是视觉云台完整比赛版。`config/profile_select.h`定义模式，两个产品的默认
功能组合分别在`config/profiles/profile_gmr.h`和`profile_full.h`；
`config/library_config.h`只定义可选方法、派生开关和依赖检查。

当前Gmr固定使用天猛星板：两路编码电机和七路数字灰度保留；UART2 PB15/PB16
连接H7并输出LCD文本、接收任务命令；UART3 PB2/PB3连接外部M0姿态模块；
本地SSD1306 OLED显示姿态。UART0日志、UART1 JY61、蓝牙和云台均默认关闭。
完整的配置归属、UART资源矩阵和新增任务步骤见`doc/PROFILES.md`，库方法目录见
`doc/COMPETITION_LIBRARY.md`。

天猛星可选的SPI1六轴模块已支持逐飞IMU660RA、IMU660RB和IMU660RC，Gmr与
Full两个Profile均默认关闭。接线、构建选择和`IMU660RX_Read()`接口见
`doc/IMU660RX.md`。

旧两车HC-05协议和调参代码仍保留在仓库历史基线中，但不进入当前Gmr构建。

Gmr菜单由`app/task_registry.c`集中声明，当前只有1项：

| 菜单项 | 实际入口 |
| --- | --- |
| Attitude | Mission1，被动显示外部M0的连续yaw、yaw角速度、序号和帧统计；不会驱动电机 |

本地K2或H7命令`@START=1\n`启动任务，长按K2或H7命令`@STOP\n`停止。
完整接口和当前保留资源见`doc/GMR_TIANMENG_CONSOLE.md`。

## Full Profile 比赛任务

显示菜单包含五个比赛入口、一个联合标定入口、一个底盘测试入口、一个云台姿态实验入口和一个编码器手推测试入口：

| 任务 | 当前流程 |
| --- | --- |
| Task 1 | 选择1～5圈后启动数字灰度循迹；支持可选的单轮锁死或双轮反转直角转向，UART1/PB7板载JY61姿态可辅助转速曲线，4次转向记1圈 |
| Task 2 | 不循迹，使用唯一point参数直接进行K230视觉追踪；yaw K由目标长度连续拟合 |
| Task 3 | 云台连续搜索，收到首个有效视觉帧后停止搜索并使用point参数追踪；yaw K由目标长度连续拟合 |
| Task 4 | 三项子菜单：追点1圈、追点2圈、追圆1圈；首帧立即发F并在500 ms后启动循迹，实际左/右转阶段暂停视觉并只用H7/JY61姿态矫正，灰度回线80 ms后恢复追踪 |
| Task 5 PID | 默认标定底盘；`mode gimbal` 调姿态环，`mode vision` 用 B 波形观察二维视觉追踪响应 |
| Task 6 Drive | 进入后选择开环/闭环和 10～100 速度；开环单位为 PWM%，闭环单位为 count/20ms，左右轮使用相同命令 |
| Task 7 Circle | 不循迹，复用Task2直接追踪流程并使用唯一circle参数；yaw K由目标长度连续拟合 |
| Task 8 IMU | H7通过UART0/PA11提供云台yaw角度与角速度反馈，板载JY61通过UART1/PB7提供底座角速度前馈；不启动底盘、视觉或Task4流程 |
| Task 9 Encoder | 关闭底盘和云台电机使能，进入时清零左右累计编码器；所选显示后端显示左右原始count以及与Task4延长段相同的平均绝对count |

按键沿用 MSPM0 原板的两键逻辑：K1 切换任务或子项，K2 确认；任务中长按 K2 停止，子菜单长按 K2 返回。

Task2/Task3/Task7不再显示距离子菜单，在主菜单按K2直接启动；距离响应由视觉帧
第5字段自动拟合。Task6参数页先选择`Closed/Open`，再选择10～100的速度，
默认闭环40；范围和步长在`config/control_config.h`修改。

## FreeRTOS 结构

系统入口在 `app/main.c`，静态任务和队列都在 `app/rtos_app.c` 创建。所有业务任务永久存在，不在比赛任务切换时动态创建或删除。

| RTOS任务 | 优先级 | 唤醒方式 | 职责 |
| --- | ---: | --- | --- |
| CarControl | 6 | 严格10 ms；灰度语义事件可提前唤醒 | 维护底盘闭环；Gmr姿态任务期间挂起并保持电机停车 |
| Gimbal | 6 | 绝对10 ms截止点；完整视觉帧可提前唤醒 | 维护H7反馈与JY61前馈、解析视觉，并运行Task4姿态辅助或Task8独占姿态控制 |
| Watchdog | 可配置 | 周期可配置 | 检查UI任务心跳；超时后停止喂WWDT0，由硬件复位整机 |
| Input | 4 | GPIO按键边沿；按住期间 1 ms | 按键消抖、长按计时并向静态事件队列投递事件 |
| Mission | 3 | 状态事件通知 | 处理任务启动、停止和状态切换 |
| Comm | 2 | 5 ms | Gmr维护H7命令与外部M0姿态；Full维护原通信链路 |
| UI | 1 | UI变化通知；20 ms板级维护 | 刷新所选H7 LCD/OLED后端；状态灯保持周期维护 |

关键调度配置：

- 抢占开启，时间片关闭：CarControl与Gimbal都按10 ms截止时间运行，其他事件任务在没有事件时阻塞。
- tick 为 1 kHz，tickless idle 关闭；空闲任务不让 MCU 进入睡眠。
- 全部任务、栈和事件队列静态分配；动态内存关闭。
- `vTaskDelete()` 关闭，任务不会在运行期被删除。
- Gmr同时开启H7 LCD文本后端和本地OLED；Full默认只开本地OLED。
- Watchdog参数集中在`config/board_config.h`的`CAR_WATCHDOG_*`宏；当前`CAR_ENABLE_UI_WATCHDOG=0`关闭监督。启用后，UI约2秒无心跳会停止喂狗，WWDT0再经过约1秒复位整机，调试器暂停内核时同步暂停。
- WWDT0违规在MSPM0G3507上产生SYSRST，不会给外部H7断电。启动日志会输出`reset cause raw=`及WWDT0、CPU LOCKUP、BOR等原因。
- Full的Gimbal任务保留绝对10 ms姿态截止点；Gmr不创建Gimbal任务，也不初始化云台STEP资源。
- Full按需使用TIMG6的20 kHz云台STEP和TIMG0的10 kHz灰度采样；当前Gmr不启动这两个定时器。TIMA0底盘20 kHz PWM不产生周期中断。
- UART3 ISR只在收到完整视觉行后唤醒Gimbal立即消费最新帧；固定10 ms姿态矫正截止点保持不变，不累积过期控制帧。
- 普通循迹灰度在10 ms控制周期边界生成一次左右目标。TIMG0仍每100 us做语义快采样，不把原始采样逐条入队；确认左/右直角或转向侧回线时提前唤醒CarControl。入弯时若有有效UART1板载JY61姿态帧就锁存航向供粗略减速参考，编码器读清、角度更新和PI仍留在固定10 ms边界。
- 普通循迹使用100 us快采样形成的确认mask；同一S1～S7完整mask连续出现2次后才更新，单次毛刺继续沿用上一确认值。

## 底盘编码电机

底盘已经从两路 STEP/DIR 步进电机改为两路 PWM 编码直流电机：

| 通道 | PWM | 方向1 | 方向2 | 编码器A | 编码器B |
| --- | --- | --- | --- | --- | --- |
| 左轮 / B | PA12 | PA29 | PA30 | PA13 | PB24 |
| 右轮 / A | PA22 | PA31 | PA28 | PB19 | PB20 |

- TIMA0 CH1/CH3 输出 20 kHz PWM。
- 编码器四路 GPIO 使用双边沿中断和正交状态表解码。
- 底盘速度环每10 ms执行，但目标和反馈保持编码器`count/20ms`刻度，范围`-100..100`；每个10 ms原始编码窗口在控制器入口乘2归一化，累计距离仍使用真实编码器`count`。
- `CAR_LIBRARY_LINE_DRIVE_METHOD`在直接PWM、PWM+编码器交叉同步和编码器速度闭环之间选择；当前默认速度闭环。
- `board_config.h`只高亮当前输出方法对应的Task1/Task4参数块，各自保存灰度权重、基准、增益和限幅。
- 选择`CAR_LIBRARY_LINE_DRIVE_PWM_ENCODER_SYNC`时，每10 ms让快轮减PWM、慢轮加PWM，并且仍只在S4确认压线时生效；选择直接PWM或S4离线时只运行灰度PWM差速。
- 直角强转和出弯始终使用目标速度闭环；正式Task4调用`MotorNoYaw_StartMission4()`并使用自己的完整profile。
- Task1/Task4 的 `LINE_LOST_TIMEOUT_MS=0` 表示丢线后持续搜线，不再因灰度全 0 自动停车；编码间距参数只限制重复强转。
- S1/S2窗口确认左直角，S6/S7窗口确认右直角；两侧窗口同时成立时不选择方向。`CAR_LIBRARY_RIGHT_ANGLE_TURN_METHOD`可选择关闭、单轮锁死或双轮反转，当前默认双轮反转。新方法保持原前进延时，Task1强转内/外轮为`-25/+25 count/20ms`，Task4为`-30/+30`；左转连续检测到S1、右转连续检测到S7后结束强转。姿态缺失不影响灰度转向；出弯内轮为`26`、外轮为`25`并同向前进40 ms。
- `config/control_config.h` 分别保存低速 `START`、正常运行 `RUN_START` 和实际 PI/前馈参数。每轮实际速度绝对值小于15 count/20ms时使用 `START`，否则使用 `RUN_START`；架空轮确认方向和编码器符号后再标定并落地测试。
- Task5 的 `pwm/set` 命令和 SerialPlot PWM 通道使用 `-100..100%`；当前 `100%=1200 PWM count`。底盘 `P` 与姿态 `A` 各为 9 通道，视觉响应 `B` 为 10 通道；`P` 的末两路持续显示左右 `FF_Q1024`，FF 始终用 raw count 计算。
- Task1/Task4 的 `TURN_MIN_ENCODER_GAP_COUNTS` 和 Task4 的`CAR_MISSION4_EXTRA_ENCODER_COUNTS=200`均为编码器 count，必须按实车重标。

## 云台与视觉

云台 yaw/pitch 继续使用 STEP/DIR：

| 轴 | STEP | DIR | EN |
| --- | --- | --- | --- |
| yaw | PA8 | PA9 | 硬件固定有效 |
| pitch | PA7 | PB18 | 硬件固定有效 |

原云台 EN 脚 PA31/PB19 已被底盘方向和编码器占用，软件 EN 接口只保留状态机兼容语义。当前配置假设驱动器 EN 低有效，实际接线需将 EN 固定到有效电平。

K230 视觉通信使用 UART3，115200 bit/s，PB2 为 MCU TX，PB3 为 MCU RX。视觉帧格式为：

```text
centerDx,centerDy;circleDx,circleDy;stageScale
```

五个字段均保留一位小数。第五字段`stageScale`现在表示目标表观长度，正常范围
`30.0~140.0`。视觉yaw增益按`K=0.8+(clamp(length,30,140)-30)*0.8/110`
连续拟合，超界分别保持0.8/1.6；pitch、H7姿态补偿、JY61前馈和固定yaw命令
不参与缩放。长度与K端点在`config/control_config.h`配置。

Full Profile的云台反馈由达妙H7板载BMI088提供。H7完成零偏、Kalman角速度滤波和Mahony姿态
更新后，从UART7/PE8输出3.3 V TTL的JY61兼容帧；M0的UART0/PA11使用独立
`h7_gyro_link`解析器，直接把yaw角度和yaw角速度送入Task4/Task8反馈环，不重复融合。
板载JY61保留在UART1/PB7，经`body_motion`提供底座yaw角速度前馈和Task1转弯辅助。
直连接线、协议和启动时序见 `doc/H7_GYRO_LINK.md`。

以下连接只适用于Full Profile。地猛星的PA11与板载CH340 TX同网，接H7前必须拆开CH340 TX。UART0全双工连接：
PA11接H7 PE8接收姿态，PA10接H7 PE7发送LCD命令和日志。H7只处理`@`开头的显示
命令并忽略普通日志。UART0文本RX关闭，不能再从Type-C接收在线调参命令。

## 灰度快路径

七路灰度为数字输入，高电平表示有效：S1～S7 对应 PA15、PA16、PA17、PA24、PA25、PA26、PA27。

当前Gmr只初始化七路数字灰度GPIO，不启动TIMG0快采样；新增正式循迹任务后再按任务需求启用采样定时器。Full仍保留原10 kHz灰度快路径和20 kHz云台STEP调度。

## 构建

在 PowerShell 中执行：

```powershell
Set-Location D:\Ti\m0-light-rtos
.\tools\build_ccs.ps1 -Profile Gmr -Clean
.\tools\build_ccs.ps1 -Profile Full -Clean
```

输出文件：

```text
D:\Ti\m0-light-rtos\Debug\profile-gmr-tianmeng-no-jy61-no-log\m0-light-rtos.out
D:\Ti\m0-light-rtos\Debug\profile-full-tianmeng\m0-light-rtos.out
```

Gmr默认关闭JY61和UART0日志，并拒绝地猛星或蓝牙组合；Full仍默认启用JY61和日志。

也可以在 CCS Theia 中导入 `D:\Ti\m0-light-rtos`。目标芯片为 MSPM0G3507，默认目标配置为 `targetConfigs\MSPM0G3507_XDS110.ccxml`。

构建不会自动下载。未明确授权时不要执行 Factory Reset、mass erase 或 NONMAIN 写入。

## 首次上板顺序

1. 断开电机动力电源，确认OLED出现唯一`Attitude`入口。
2. 确认H7 UART2交叉接线，并验证H7能收到LCD文本、发出启动/停止命令。
3. 接入外部M0 UART3，确认OLED的yaw、角速度、序号和帧计数持续变化。
4. 架空左右轮，确认姿态任务全程不会驱动电机；正式运动任务加入后再做电机标定。

完整接线见 `doc/PINOUT.md`。

旧底盘调参资料仍保留在`doc/CONTROL_TUNING.md`，但不属于当前唯一任务。
