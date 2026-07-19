# m0-light-rtos

MSPM0G3507 光电小车工程，使用 CCS Theia、TI Arm Clang 4.0.4 LTS 和 FreeRTOS V11.3.0。

工程目录：

```text
D:\Ti\m0-light-rtos
```

本工程以 `D:\Ti\light-car1.0ccs` 的比赛 Task1～4、菜单、视觉通信和云台控制为业务基础，移植 `Trace-f407` 已整理的 FreeRTOS 分层。原工程保持不变。

## 比赛任务

H7 LCD菜单包含五个比赛入口、一个联合标定入口、一个底盘测试入口、一个云台姿态实验入口和一个编码器手推测试入口：

| 任务 | 当前流程 |
| --- | --- |
| Task 1 | 选择1～5圈后启动数字灰度循迹；支持左右直角识别，转向侧最外传感器回线确认完成，UART1/PB7板载JY61姿态仅作可选角度辅助，4次转向记1圈 |
| Task 2 | 选择 Near/Mid/Far；不循迹，按对应中心参数直接进行 K230 视觉追踪 |
| Task 3 | 选择 Near/Mid/Far；云台以当前搜索速度连续匀速转动，收到首个有效视觉帧后立即停止搜索并按对应中心参数追踪 |
| Task 4 | 三项子菜单：追点1圈、追点2圈、追圆1圈；首帧后立即追踪并启动Task1循迹，H7反馈矫正叠加到视觉yaw，JY61仅在转弯阶段做底座角速度前馈；目标丢失后以400 SPS在丢失位置±400 STEP内左右摆动重搜 |
| Task 5 PID | 默认标定底盘；`mode gimbal` 调姿态环，`mode vision` 用 B 波形观察二维视觉追踪响应 |
| Task 6 Drive | 进入后选择开环/闭环和 10～100 速度；开环单位为 PWM%，闭环单位为 count/20ms，左右轮使用相同命令 |
| Task 7 Circle | 选择 Near/Mid/Far；不循迹，复用 Task2 的直接追踪流程，但使用对应距离的圆点参数 |
| Task 8 IMU | H7通过UART0/PA11提供云台yaw角度与角速度反馈，板载JY61通过UART1/PB7提供底座角速度前馈；不启动底盘、视觉或Task4流程 |
| Task 9 Encoder | 关闭底盘和云台电机使能，进入时清零左右累计编码器；H7 LCD显示左右原始count以及与Task4延长段相同的平均绝对count |

按键沿用 MSPM0 原板的两键逻辑：K1 切换任务或子项，K2 确认；任务中长按 K2 停止，子菜单长按 K2 返回。

Task2/Task3/Task7 参数页使用 `Near/Mid/Far` 三档，K1 切换距离，K2 启动；
Task2/Task3 使用对应中心参数，Task7 使用对应圆点参数。Task6 参数页先选择
`Closed/Open`，再选择 10～100 的速度，默认闭环 40；范围和步长在
`config/control_config.h` 修改。

## FreeRTOS 结构

系统入口在 `app/main.c`，静态任务和队列都在 `app/rtos_app.c` 创建。所有业务任务永久存在，不在比赛任务切换时动态创建或删除。

| RTOS任务 | 优先级 | 唤醒方式 | 职责 |
| --- | ---: | --- | --- |
| CarControl | 6 | 严格 20 ms；灰度语义事件可提前唤醒 | Task1/Task4-LINE 依次采灰度、计算目标，再读取清零编码器窗口、执行速度 PI、更新底盘 PWM |
| Gimbal | 6 | 严格 10 ms `xTaskDelayUntil` | 维护H7反馈与JY61前馈、解析视觉，并运行Task4姿态辅助或Task8独占姿态控制 |
| Watchdog | 可配置 | 周期可配置 | 检查UI任务心跳；超时后停止喂WWDT0，由硬件复位整机 |
| Input | 4 | GPIO按键边沿；按住期间 1 ms | 按键消抖、长按计时并向静态事件队列投递事件 |
| Mission | 3 | 任务/视觉通知；Task3连续搜索和Task4非循迹阶段 1 ms | 处理比赛状态切换及非底盘周期流程；Task1/Task4-LINE 交给 CarControl |
| Comm | 2 | 5 ms | UART Link状态、Task5串口命令和低频日志维护 |
| UI | 1 | UI变化通知；20 ms板级维护 | H7 LCD只发送变化行；状态灯保持周期维护 |

关键调度配置：

- 抢占开启，时间片关闭：CarControl按20 ms截止时间运行；Gimbal固定10 ms运行，其他事件任务在没有事件时阻塞。
- tick 为 1 kHz，tickless idle 关闭；空闲任务不让 MCU 进入睡眠。
- 全部任务、栈和事件队列静态分配；动态内存关闭。
- `vTaskDelete()` 关闭，任务不会在运行期被删除。
- 本地OLED不再初始化；菜单、任务状态、启动进度和Task5动态页经UART0/PA10发送`@L0`~`@L9`固定行命令到H7 LCD。
- Watchdog参数集中在`config/board_config.h`的`CAR_WATCHDOG_*`宏；当前`CAR_ENABLE_UI_WATCHDOG=0`关闭监督。启用后，UI约2秒无心跳会停止喂狗，WWDT0再经过约1秒复位整机，调试器暂停内核时同步暂停。
- WWDT0违规在MSPM0G3507上产生SYSRST，不会给外部H7断电。启动日志会输出`reset cause raw=`及WWDT0、CPU LOCKUP、BOR等原因。
- Gimbal任务始终以10 ms运行。H7的已处理yaw/角速度直接进入反馈环，板载JY61只在`body_motion`中做底座角速度前馈估计。Task4直线阶段关闭JY61前馈但保留H7反馈，灰度确认进入转弯流程后才开启前馈；视觉主动纠偏时H7姿态参考同步跟随，避免两个闭环互相抵消。Task1/Task6没有云台命令时STEP定时器保持停止。Task2/Task3/Task7/Task8进入后停车并挂起CarControl；Task8不启动视觉。Task6保留CarControl以刷新编码器反馈或运行20ms速度环。
- TIMG6按需承担20 kHz云台STEP；TIMG0只在Task1/Task4循迹时承担10 kHz灰度采样。TIMA0底盘20 kHz PWM不产生周期中断。
- UART3 ISR只在收到完整视觉行后记录通知；Gimbal在下一次固定10 ms边界消费最新帧，不累积过期控制帧。
- 普通循迹灰度只在20 ms控制周期边界生成一次左右目标。TIMG0仍每100 us做语义快采样，不把原始采样逐条入队；确认左/右直角或转向侧回线时提前唤醒CarControl。入弯时若有有效UART1板载JY61姿态帧就锁存航向供粗略减速参考，编码器读清、角度更新和PI仍留在固定20 ms边界。
- 普通循迹使用100 us快采样形成的确认mask；同一S1～S7完整mask连续出现2次后才更新，单次毛刺继续沿用上一确认值。

## 底盘编码电机

底盘已经从两路 STEP/DIR 步进电机改为两路 PWM 编码直流电机：

| 通道 | PWM | 方向1 | 方向2 | 编码器A | 编码器B |
| --- | --- | --- | --- | --- | --- |
| 左轮 / A | PA22 | PA31 | PA28 | PB19 | PB20 |
| 右轮 / B | PA12 | PA21 | PA23 | PA13 | PB24 |

- TIMA0 CH1/CH3 输出 20 kHz PWM。
- 编码器四路 GPIO 使用双边沿中断和正交状态表解码。
- 底盘速度环内部目标单位是编码器 `count/20ms`，范围 `-100..100`；累计距离单位是编码器 `count`。
- Task1/Task4 NO YAW 配置与 Task5 `target/move` 直接使用相同的有符号 `count/20ms` 刻度；正式Task4直接调用Task1的`MotorNoYaw_Start()`循迹入口。
- 正常闭环收到左右相同的非零目标时，会按本周期左右编码器反馈相对平均速度的偏差，对两侧有效目标做等量反向修正；原始平均目标保持不变。修正增益和单侧上限在 `config/control_config.h`，有灰度差速、单轮转弯及Task5独立标定时不启用。
- Task1/Task4 的 `LINE_LOST_TIMEOUT_MS=0` 表示丢线后持续搜线，不再因灰度全 0 自动停车；编码间距参数只限制重复强转。
- S1/S2窗口确认左直角，S6/S7窗口确认右直角；两侧窗口同时成立时不选择方向。强转内轮目标为`-1 count/20ms`，直接由正常闭环给反向作用；外轮当前从`10`渐变到`8`。姿态缺失不影响灰度转向。左转等待S1先释放再回线，右转等待S7先释放再回线；出弯外轮为`8`、内轮为`12`并保持40 ms。持续5秒仍找不到回线才安全停车。
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

五个字段均保留一位小数。第五字段`stageScale`严格大于`60.0`时，视觉产生的yaw
命令及对应速度上限乘`K=1.4`；等于`60.0`不放大。pitch、H7姿态补偿和JY61前馈
不参与该倍增。阈值和K在`config/control_config.h`配置。

云台反馈由达妙H7板载BMI088提供。H7完成零偏、Kalman角速度滤波和Mahony姿态
更新后，从UART7/PE8输出3.3 V TTL的JY61兼容帧；M0的UART0/PA11使用独立
`h7_gyro_link`解析器，直接把yaw角度和yaw角速度送入Task4/Task8反馈环，不重复融合。
板载JY61保留在UART1/PB7，经`body_motion`提供底座yaw角速度前馈和Task1转弯辅助。
直连接线、协议和启动时序见 `doc/H7_GYRO_LINK.md`。

地猛星的PA11与板载CH340 TX同网，接H7前必须拆开CH340 TX。UART0全双工连接：
PA11接H7 PE8接收姿态，PA10接H7 PE7发送LCD命令和日志。H7只处理`@`开头的显示
命令并忽略普通日志。UART0文本RX关闭，不能再从Type-C接收在线调参命令。

## 灰度快路径

七路灰度为数字输入，高电平表示有效：S1～S7 对应 PA15、PA16、PA17、PA24、PA25、PA26、PA27。

TIMG0 以 10 kHz（100 us）进入灰度采样中断；云台 STEP 由独立的 TIMG6 以 20 kHz 调度。`MotorNoYaw_TimerSample()` 只读取 GPIO、更新小计数器和置事件标志；仅当形成右转/回线语义事件时使用 ISR-safe Task Notification 唤醒 CarControl，不调用 OLED 或日志，也不会在不足20 ms时提前清零编码器窗口。

## 构建

在 PowerShell 中执行：

```powershell
Set-Location D:\Ti\m0-light-rtos
.\tools\build_ccs.ps1 -Clean
```

输出文件：

```text
D:\Ti\m0-light-rtos\Debug\codex-build\m0-light-rtos.out
```

也可以在 CCS Theia 中导入 `D:\Ti\m0-light-rtos`。目标芯片为 MSPM0G3507，默认目标配置为 `targetConfigs\MSPM0G3507_XDS110.ccxml`。

构建不会自动下载。未明确授权时不要执行 Factory Reset、mass erase 或 NONMAIN 写入。

## 首次上板顺序

1. 断开电机动力电源，只确认H7 LCD、按键、UART和任务菜单。
2. 保持电机动力断开，分别确认UART0/PA11的H7反馈和UART1/PB7的JY61前馈持续刷新；手动跨过H7 yaw的±180度检查连续展开。
3. 架空左右轮，先用低限幅验证 A/B 电机方向和编码器正负号。
4. 标定每侧 `FF`，再调 `KP/KI`，最后提高目标速度。
5. 接地先单独测试一个右弯，确认S4先离线再重新连续命中时才出弯；同时断开UART1/PB7的JY61，验证小车仍会靠灰度完成转弯，不会半途停车。
6. 最后测 Task1 一圈，再测 Task2～4。

完整接线见 `doc/PINOUT.md`。

Task5 串口命令、公式和完整标定步骤见 `doc/CONTROL_TUNING.md`。
