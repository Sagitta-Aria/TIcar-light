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

当前Gmr固定使用天猛星板：两路编码电机和八路数字红外巡线；UART2 PB15单向
连接H7并发送LCD文本和控制请求；UART3 PB2/PB3连接外部M0姿态模块；
本地SSD1306 OLED显示姿态。UART0日志、UART1 JY61、蓝牙和云台均默认关闭。
完整的配置归属、UART资源矩阵和新增任务步骤见`doc/PROFILES.md`，库方法目录见
`doc/COMPETITION_LIBRARY.md`。

天猛星可选的SPI1六轴模块已支持逐飞IMU660RA、IMU660RB和IMU660RC，Gmr与
Full两个Profile均默认关闭。接线、构建选择和`IMU660RX_Read()`接口见
`doc/IMU660RX.md`。Gmr的IR8已使用PB9，因此Gmr不能同时启用IMU660RX。

旧两车HC-05协议和调参代码仍保留在仓库历史基线中，但不进入当前Gmr构建。

Gmr任务仍由`app/task_registry.c`集中声明。原Mission1～9调试入口保持不变，并新增
Mission10～13承载正式Task3/4/5和BMI Y前馈测试：

| 菜单项 | 实际入口 |
| --- | --- |
| Attitude | Mission1，被动显示外部M0的连续yaw、yaw角速度、序号和帧统计；不会驱动电机 |
| Task2 A-A | Mission2，基础速度从12在2秒内升到50 count/20ms，连续使用八路红外加权差速循迹；左右停车count可在Flash菜单中独立调节 |
| Encoder | Mission3，进入时清零左右累计编码器count，随后只读显示手推产生的带符号count；不会驱动电机 |
| Drive Adjustable | Mission4，左右目标可分别配置，默认15/27 count/20ms |
| Direction +20 | Mission5，在开环+20%与闭环+20 count/20ms之间检查方向 |
| IR Differential | Mission6，八路红外加权差速并在丢线后保持最后目标2000 ms |
| Line Follow | Mission7，连续使用八路红外加权差速循迹，运行到主动停止 |
| H7 BMI Ball | Mission8，保持M0底盘停车，通过UART2向H7发送一次可选启动请求；BMI088、视觉、滚球控制和LCD状态均由H7独立执行 |
| H7 Step Test | Mission9，向H7发送步进电机绝对位置测试命令 |
| Task3 Ball | Mission10，底盘停车，发送`@BALL=TASK3`，由H7接收视觉帧并使用Task3独立参数滚球 |
| Task4 Track+Ball | Mission11，清零编码器，以10 (count/20ms)/s加速并使用Task4独立八路红外循迹；接近终点后同比减速，同时发送`@BALL=TASK4`启动H7滚球 |
| Task5 Track+Ball | Mission12，清零编码器，以10 (count/20ms)/s加速并使用Task5独立八路红外循迹；接近终点后同比减速，同时发送`@BALL=TASK5`启动H7滚球 |
| BMI Y FF Test | Mission13，位于OTHER，发送`@BALL=IMUY`测试H7车体Y向线性加速度位置前馈 |

顶层菜单只显示`TASK`和`OTHER`。`TASK`保留原Task2循迹A-A和BMI滚球调试两项，
并追加Task3视觉滚球、Task4循迹+滚球和Task5循迹+滚球；`OTHER`保留原姿态、
编码器、底盘、红外和步进测试，并追加BMI Y前馈测试及`Stop Count Flash`参数页。
普通菜单中K1下一项、K4上一项、K2进入或启动、K3逐级返回；`Stop Count Flash`
中K1加100、K4减100、K2依次切换六项并在最后一项写入Flash，K3取消本次修改。
所有按键均为短按。
完整接口和当前保留资源见`doc/GMR_TIANMENG_CONSOLE.md`。

Mission8启动时M0只发送一次`@BALL=BMI\n`，停止、退出或错误时只发送一次
`@BALL=STOP\n`；发送失败可由通信任务重试，但成功发送后没有周期心跳。H7不依赖
M0保活，USART10视觉帧和BMI088数据也不经过M0。M0仅提供共享菜单、可选启停请求
和其他既有单向输出。Mission8运行或停止页由H7本地刷新，M0不等待确认，也不再
发送LCD文本覆盖H7状态页；回到主菜单后恢复共享菜单显示。

Mission10～13沿用同一生命周期，但分别发送`@BALL=TASK3`、`@BALL=TASK4`、
`@BALL=TASK5`和`@BALL=IMUY`。Task4/5的循迹配置与TASK子菜单第1项完全分离，
三套宏均位于`config/gmr_task_line_follow_config.h`；当前数值相同只是安全初值，
可在不影响其他任务的前提下分别标定。Task1、Task4、Task5各自的左右停车count
也分别占用独立Flash槽位；`GMR_MISSION2_*_STOP_COUNT`及
`GMR_TASK4/5_LINE_FOLLOW_*_STOP_COUNT`只提供Flash无效时的默认值。记录保存在
MSPM0G3507主Flash最后1 KB扇区，带版本、范围、CRC和写后校验。Task4/5在左右
剩余里程都小于`GMR_TASK45_LINE_FOLLOW_DECEL_LEAD_COUNTS`后进入减速；每拍先计算
灰度左右目标，再乘同一个递减比例。速度降到0后进入`FINISHED`并停止底盘，但不
发送`@BALL=STOP`，H7继续平衡球；之后人工退出、停止或切换任务时才停止H7滚球。

## Full Profile 比赛任务

显示菜单包含五个比赛入口、一个联合标定入口、一个底盘测试入口、一个云台姿态实验入口和一个编码器手推测试入口：

| 任务 | 当前流程 |
| --- | --- |
| Task 1 | 启动数字灰度差速循迹，不再识别或执行独立直角强转；持续运行到按K3停止 |
| Task 2 | 不循迹，使用唯一point参数直接进行K230视觉追踪；yaw K由目标长度连续拟合 |
| Task 3 | 云台连续搜索，收到首个有效视觉帧后停止搜索并使用point参数追踪；yaw K由目标长度连续拟合 |
| Task 4 | 选择point/circle视觉源；首帧立即发F并在500 ms后启动连续灰度循迹，不再进入独立直角强转 |
| Task 5 PID | 默认标定底盘；`mode gimbal` 调姿态环，`mode vision` 用 B 波形观察二维视觉追踪响应 |
| Task 6 Drive | 进入后选择开环/闭环和 10～100 速度；开环单位为 PWM%，闭环单位为 count/20ms，左右轮使用相同命令 |
| Task 7 Circle | 不循迹，复用Task2直接追踪流程并使用唯一circle参数；yaw K由目标长度连续拟合 |
| Task 8 IMU | H7通过UART0/PA11提供云台yaw角度与角速度反馈，板载JY61通过UART1/PB7提供底座角速度前馈；不启动底盘、视觉或Task4流程 |
| Task 9 Encoder | 关闭底盘和云台电机使能，进入时清零左右累计编码器；所选显示后端显示左右原始count以及与Task4延长段相同的平均绝对count |

天猛星按键为：K1 下一项，K4 上一项，K2 确认/启动，K3 退出/停止；所有按键均为短按。

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
| Input | 4 | GPIO按键边沿；按住期间 1 ms | 按键释放时向静态事件队列投递短按事件 |
| Mission | 3 | 状态事件通知 | 处理任务启动、停止和状态切换 |
| Comm | 2 | 5 ms | Gmr发送H7命令并维护外部M0姿态；Full维护原通信链路 |
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
- 普通循迹灰度在10 ms控制周期边界生成一次左右目标。TIMG0每1 ms读取一次数字mask，不再生成直角入弯或回线事件。
- 同一S1～S7完整mask连续出现2次后才更新，单次毛刺继续沿用上一确认值。

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
- Task1/Task4 的 `LINE_LOST_TIMEOUT_MS=0` 表示丢线后持续沿用上一拍或搜线，不因灰度全0自动停车。
- `CAR_LIBRARY_RIGHT_ANGLE_TURN_METHOD`在Gmr和Full默认均为`NONE`；S1～S7只参与普通加权差速，不再切换到强转或出弯状态。
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

## 循迹传感器

当前Gmr默认初始化八路红外模块。IR1～IR8从车头朝前按左到右排列，对应
PA15、PA16、PA17、PA24、PA25、PA26、PA27、PB9。按模块例程，高电平表示
黑线、低电平表示白底；读取结果的bit7～bit0依次对应IR1～IR8。

PB9原为IMU660RX的SPI1 SCK，Gmr已关闭该陀螺仪，编译期也会拒绝同时启用八路
红外与IMU660RX。Full Profile仍可选择原七路数字灰度方案。

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

1. 断开电机动力电源，确认主菜单可进入`TASK`和`OTHER`并滚动显示全部入口。
2. 确认M0 PB15 TX接H7 PE7 RX并共地，验证H7能收到LCD文本和控制请求；PB16不用连接。
3. 接入外部M0 UART3，确认OLED的yaw、角速度、序号和帧计数持续变化。
4. 保持电机动力断开，在`Encoder`页手推轮子，确认左右count按实际方向变化。
5. 架空左右轮并准备物理断电，在`Drive Adjustable`页确认目标、反馈和PWM方向；确认无误后再落地测试。
6. 在`H7 BMI Ball`页按K2，确认H7本地状态页接管LCD并运行；按K3后确认H7步进电机停止。

完整接线见 `doc/PINOUT.md`。

旧底盘调参资料仍保留在`doc/CONTROL_TUNING.md`，但不属于当前Gmr任务菜单。
