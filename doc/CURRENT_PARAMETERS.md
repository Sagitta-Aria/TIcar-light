# Full历史参数快照

> 本文内容早于2026-07-29天猛星Gmr任务清理，仅作为Full和旧控制算法参数参考，
> 不是当前Gmr发布配置。当前Gmr资源、任务和协议以`GMR_TIANMENG_CONSOLE.md`为准。

- 快照日期：2026-07-21
- 目标芯片：MSPM0G3507
- 工具链：TI Arm Clang 4.0.4 LTS
- 工程目录：`D:\Ti\m0-light-rtos`

本文保存当前源码实际选择并生效的参数，供后续调赛道、调电机或切换库方法时对照。
它是快照，不是新的配置入口；真正修改参数仍应回到以下文件：

| 参数类别 | 配置源 |
| --- | --- |
| 库方法选择 | `config/library_config.h` |
| 赛道、循迹、转向、显示和任务参数 | `config/board_config.h` |
| 底盘内环、视觉缩放和双IMU参数 | `config/control_config.h` |
| point/circle视觉云台参数 | `app/staticconfig.c` |
| 引脚映射 | `config/pin_map.h`、`generated/ti_msp_dl_config.h` |

> 单位约定：底盘速度默认使用 `encoder count/20ms`；云台速度使用 `step/s`；
> PWM使用TIMA0定时器原始count；`Q1024`中1024表示1.0。

## 当前库组合

| 类别 | 当前选择 | 状态 |
| --- | --- | --- |
| 主控板引脚 | `CAR_LIBRARY_BOARD_DIMENG_48P` | 地猛星48P为当前默认 |
| 灰度输入 | `CAR_LIBRARY_GRAY_INPUT_DIGITAL_GPIO_7` | 开启，7路GPIO数字灰度 |
| 循迹算法 | `CAR_LIBRARY_LINE_FOLLOW_DIGITAL_WEIGHTED` | 开启，数字量加权 |
| 循迹底盘输出 | `CAR_LIBRARY_LINE_DRIVE_ENCODER_SPEED_LOOP` | 开启，编码器速度闭环 |
| 直角转向 | `CAR_LIBRARY_RIGHT_ANGLE_TURN_COUNTER_ROTATE` | 开启，内外轮反向 |
| 强转速度 | `CAR_LIBRARY_TURN_SPEED_JY61_ANGLE_PROFILE` | 开启，JY61转角曲线 |
| 二维激光云台 | `CAR_LIBRARY_GIMBAL_TRACKING_2D_LASER_PD_FF` | 开启 |
| 云台姿态矫正 | `CAR_LIBRARY_GIMBAL_ATTITUDE_H7_JY61_DUAL_IMU` | 开启 |
| 云台起步搜点 | `CAR_LIBRARY_GIMBAL_STARTUP_SEARCH_FIXED_YAW` | 开启 |
| 云台丢目标恢复 | `CAR_LIBRARY_GIMBAL_LOST_TARGET_NONE` | 关闭 |
| Task4强转yaw随动 | `CAR_LIBRARY_GIMBAL_TURN_FOLLOW_NONE` | 关闭 |
| H7 UART LCD | `CAR_LIBRARY_H7_LCD_UART_TEXT` | 开启 |
| 本地I2C OLED | `CAR_LIBRARY_LOCAL_OLED_I2C_SSD1306` | 开启 |

当前两块显示屏同时开启，同一份菜单会同步输出到H7 LCD和本地OLED。

## 系统与板级开关

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| `CAR_RECOVERY_SAFE_BUILD` | 0 | 正常比赛固件 |
| `CAR_ENABLE_LOG_UART` | 1 | 开启UART0普通日志 |
| `CAR_ENABLE_LOG_UART_RX` | 0 | 关闭UART0文本接收，PA11留给H7姿态 |
| `CAR_ENABLE_DEBUG_LED` | 1 | 开启当前板型状态灯；地猛星PA14 |
| `CAR_ENABLE_UI_WATCHDOG` | 0 | 当前不启动UI监督和WWDT0 |
| `CAR_WATCHDOG_TASK_PRIORITY` | 5 | 看门狗任务优先级 |
| `CAR_WATCHDOG_TASK_STACK_WORDS` | 96 word | 384字节任务栈 |
| `CAR_WATCHDOG_CHECK_PERIOD_MS` | 250 ms | UI心跳检查周期 |
| `CAR_WATCHDOG_UI_TIMEOUT_MS` | 2000 ms | UI无心跳判定时间 |
| `CAR_WATCHDOG_HW_CLOCK_DIVIDER` | `DIVIDE_8` | WWDT时钟8分频 |
| `CAR_WATCHDOG_HW_TIMER_PERIOD` | `12_BITS` | 硬件超时约1秒 |
| `CAR_CHASSIS_LEFT_REVERSE` | 0 | 左轮命令方向不反转 |
| `CAR_CHASSIS_RIGHT_REVERSE` | 1 | 右轮命令方向反转 |
| `CAR_KEY_ACTIVE_LOW` | 1 | K1~K4低电平按下，K5仅保留 |

## 显示参数

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| H7 LCD方法 | UART0文本行 | PA10发送`@L0`至`@L9`和`@CLEAR` |
| 本地OLED方法 | I2C SSD1306 | I2C0，默认地址0x3C，失败尝试0x3D |
| `CAR_LOCAL_OLED_FONT_SIZE_PIXELS` | 12 px | OLED ASCII字体高度 |
| `CAR_LOCAL_OLED_VISIBLE_ROWS` | 5行 | 由64/12向下取整 |
| `CAR_LOCAL_OLED_MAX_CHARS_PER_ROW` | 20字符 | 防止OLED自动换行 |
| `CAR_MENU_REFRESH_MS` | 100 ms | 常规菜单刷新间隔 |

## 灰度输入与判定

### 数字灰度硬件

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| `GRAY_SENSOR_COUNT` | 7 | S1至S7 |
| `GRAY_DIGITAL_ACTIVE_HIGH` | 1 | 高电平表示压线 |
| `GRAY_DIGITAL_INPUT_PULL_UP` | 1 | MCU内部上拉开启 |
| `CAR_GRAY_TRACK_SENSOR_MASK` | `0x7F` | S1至S7全部参与循迹 |
| `GRAY_DIGITAL_CONFIRM_COUNT` | 1次 | 单路数字状态不额外防抖 |
| `GRAY_LINE_ERROR_SCALE` | 100 | 加权误差整数缩放 |

### 直角和回线检测

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| `CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US` | 1000 us | 灰度语义采样间隔 |
| `CAR_MOTOR_NO_YAW_LEFT_TURN_WINDOW_MS` | 100 ms | S1/S2左直角组合窗口 |
| `CAR_MOTOR_NO_YAW_RIGHT_TURN_WINDOW_MS` | 100 ms | S6/S7右直角组合窗口 |
| `CAR_MOTOR_NO_YAW_LINE_CONFIRM_SAMPLES` | 2次 | 完整mask连续确认次数 |
| `CAR_MOTOR_NO_YAW_TURN_REARM_MS` | 20 ms | 下一次直角检测重新开门时间 |
| `CAR_MOTOR_NO_YAW_RETURN_CONFIRM_SAMPLES` | 2次 | S1或S7回线确认次数 |
| `CAR_MOTOR_NO_YAW_TURN_TARGET_ANGLE_X100` | 7500 | JY61外轮速度曲线参考角75.00度 |

## Task1数字灰度循迹

速度单位均为 `count/20ms`。

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| 基础速度 | 30 | S4居中时左右轮目标 |
| 最低单轮速度 | -5 | 灰度修正后的下限 |
| 最高单轮速度 | 35 | 灰度修正后的上限 |
| 灰度死区 | 1 | `lineError`死区 |
| 灰度P增益 | 3 | 当前偏差修正 |
| 灰度D增益 | 0 | 当前关闭D项 |
| 单轮修正上限 | 30 | 最大差速修正绝对值 |
| 默认搜线速度 | 10 | 启动即丢线且无上一拍时使用 |
| S1至S7权重 | `13, 11, 5, 0, -5, -11, -13` | 正值左转，负值右转 |

## Task4数字灰度循迹

速度单位均为 `count/20ms`。

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| 基础速度 | 20 | S4居中时左右轮目标 |
| 最低单轮速度 | -5 | 灰度修正后的下限 |
| 最高单轮速度 | 35 | 灰度修正后的上限 |
| 灰度死区 | 1 | `lineError`死区 |
| 灰度P增益 | 3 | 当前偏差修正 |
| 灰度D增益 | 0 | 当前关闭D项 |
| 单轮修正上限 | 30 | 最大差速修正绝对值 |
| 默认搜线速度 | 10 | 启动即丢线且无上一拍时使用 |
| S1至S7权重 | `13, 11, 3, 0, -3, -11, -13` | Task4独立权重 |

## 直角转向（当前关闭）

当前两个Profile均选择`CAR_LIBRARY_RIGHT_ANGLE_TURN_NONE`。以下数值是旧方法的
历史参考，不参与当前固件的左右轮输出。

### Task1转向参数

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| 强转内轮速度 | -25 | 负数表示内轮后退 |
| 左转外轮初始速度 | 25 | 左转时右轮前进 |
| 左转外轮接近速度 | 25 | 接近75度参考角时速度 |
| 右转外轮初始速度 | 25 | 右转时左轮前进 |
| 右转外轮接近速度 | 25 | 接近75度参考角时速度 |
| 两次强转最小编码器间隔 | 3500 count | 左右绝对增量平均值 |
| 入弯继续前进时间 | 100 ms | 确认直角后沿用上一拍循迹命令 |
| 出弯外轮速度 | 25 | S1/S7回线后同向前进 |
| 出弯内轮速度 | 26 | 略快于外轮，帮助摆正 |
| 出弯保持时间 | 40 ms | 之后恢复普通循迹 |
| 强转安全超时 | 10000 ms | 未回线则停车 |
| 普通丢线停车时间 | 0 ms | 0表示持续搜线，不自动停车 |

### Task4转向参数

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| 强转内轮速度 | -30 | 负数表示内轮后退 |
| 左转外轮初始速度 | 30 | 左转时右轮前进 |
| 左转外轮接近速度 | 30 | 接近75度参考角时速度 |
| 右转外轮初始速度 | 30 | 右转时左轮前进 |
| 右转外轮接近速度 | 30 | 接近75度参考角时速度 |
| 两次强转最小编码器间隔 | 3500 count | 左右绝对增量平均值 |
| 入弯继续前进时间 | 200 ms | 确认直角后沿用上一拍循迹命令 |
| 出弯外轮速度 | 25 | S1/S7回线后同向前进 |
| 出弯内轮速度 | 26 | 略快于外轮，帮助摆正 |
| 出弯保持时间 | 40 ms | 之后恢复普通循迹 |
| 强转安全超时 | 5000 ms | 未回线则停车 |
| 普通丢线停车时间 | 0 ms | 0表示持续搜线，不自动停车 |

## Task4任务参数

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| `CAR_MISSION4_LASER_TO_LINE_DELAY_MS` | 500 ms | 首帧发F后等待再启动循迹 |
| `CAR_MISSION4_GIMBAL_CORRECTION_RELEASE_DELAY_MS` | 80 ms | 回线后继续姿态矫正并屏蔽视觉 |
| `CAR_MISSION4_GIMBAL_PITCH_EXIT_STEPS` | 100 step | 每次退出姿态矫正后pitch固定向上补偿 |
| `CAR_MISSION4_GIMBAL_PITCH_EXIT_SPEED_SPS` | 400 step/s | pitch退出补偿速度 |
| `CAR_MISSION4_EXTRA_ENCODER_COUNTS` | 3500 count | 完成目标圈数后的继续前进距离 |
| 固定yaw基础速度 | 0 step/s | 强转固定yaw随动库当前关闭 |
| 丢目标搜索速度/幅度 | 0 / 0 | 丢目标恢复库当前关闭 |

## 底盘PWM与速度环

### 公共时基和限幅

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| PWM周期 | 1600 count | 32 MHz / 1600 = 20 kHz |
| PWM硬件最大值 | 1599 count | 定时器比较值上限 |
| PWM软件限幅 | 1200 count | Task5中对应100% |
| 速度环执行周期 | 10 ms | 每10ms读清编码器窗口并运行PI |
| 对外速度单位 | count/20ms | 所有目标和反馈统一刻度 |
| CPS兼容接口上限 | 5000 count/s | 等于100 count/20ms |
| 目标速度上限 | 100 count/20ms | 单轮目标绝对值上限 |
| Q1024基准 | 1024 | 表示1.0 |

### 零速阻尼和直线同步

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| 零目标阻尼 | 开启 | 仅上层显式允许时生效 |
| 零速阻尼死区 | 0 count/20ms | 非零反馈即可参与制动 |
| 零速阻尼Kp | 2048 Q1024 | 实际2.0 |
| 零速阻尼PWM上限 | 300 count | 防止突然反冲 |
| 直线同步增益 | 1024 Q1024 | 实际1.0 |
| 直线同步修正上限 | 5 count/20ms | 单轮目标修正上限 |

### 左右轮已存PI/前馈

| 参数 | 左轮 | 右轮 | 单位/说明 |
| --- | ---: | ---: | --- |
| 起步切换阈值 | 15 | 15 | count/20ms，共用阈值 |
| `START_PWM` | 200 | 160 | 停车或换向起步基础PWM |
| `RUN_START_PWM` | 180 | 140 | 轮子已转动后的基础PWM |
| `KP_Q1024` | 16000 | 6200 | 比例增益 |
| `KI_Q1024` | 100 | 100 | 积分增益 |
| `FF_Q1024` | 12000 | 4096 | 每1 count/20ms的PWM前馈 |
| 积分输出上限 | 360 | 360 | PWM count |
| 编码器方向符号 | -1 | 1 | 左轮反转、右轮保持 |

整车同向起步积分预补偿为0 PWM count，当前未启用左右差速预装。

### Task6默认值

| 参数 | 当前值 |
| --- | ---: |
| 菜单最小值 | 10 |
| 菜单最大值 | 100 |
| 菜单步长 | 10 |
| 默认模式 | 闭环 |
| 闭环默认速度 | 40 count/20ms |
| 开环默认速度 | 40% PWM |

## 云台步进公共参数

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| 最大速度 | 1600 step/s | yaw/pitch公共目标上限 |
| STEP高电平 | 1个TIMG6 tick | 当前50 us |
| 斜坡更新周期 | 1 ms | 每1ms更新当前速度 |
| 默认加速度步长 | 80 step/s/次 | 每1ms最多增加80 |
| 默认减速度步长 | 80 step/s/次 | 每1ms最多减少80 |
| 起步搜点yaw速度 | 400 step/s | 固定yaw搜点方法 |
| 云台默认请求使能 | 0 | 上电不自动进入闭环 |
| yaw方向反转 | 0 | 保持计算方向 |
| pitch方向反转 | 0 | 保持计算方向 |
| 视觉超时 | 60 ms | 超时停止视觉追点 |
| pitch软限位 | +/-600 step | 相对每次启用位置 |

## 二维视觉云台参数

视觉误差单位为0.1像素，速度单位为step/s。控制公式使用整数增益并统一除以
`gainScale=100`。

| 参数 | Point中心点 | Circle圆点 |
| --- | ---: | ---: |
| 误差来源 | `centerDx/centerDy` | `circleDx/circleDy` |
| X/Y停止死区 | 0 / 0 | 0 / 0 |
| X/Y重新启动死区 | 12 / 12 | 12 / 12 |
| X/Y Kp | 250 / 100 | 200 / 20 |
| X/Y Kd | 40 / 20 | 0 / 0 |
| X/Y趋势前馈Kff | 75 / 25 | 50 / 5 |
| 增益除数 | 100 | 100 |
| X/Y最小速度 | 0 / 0 | 0 / 0 |
| X/Y最大速度 | 500 / 400 | 500 / 400 |
| X/Y安装补偿 | 0 / 0 | 0 / 0 |

### 目标长度到yaw增益

| 参数 | 当前值 | 实际含义 |
| --- | ---: | --- |
| 最小目标长度 | 300 | 30.0 |
| 最大目标长度 | 1400 | 140.0 |
| 最小yaw增益 | 819 Q1024 | 约0.8倍 |
| 最大yaw增益 | 1638 Q1024 | 约1.6倍 |

超出30.0至140.0范围先钳位，再线性拟合yaw增益。pitch、姿态补偿和固定yaw
命令不参与该缩放。

## 双IMU云台姿态矫正

### 板载JY61底座估计

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| 固定控制周期 | 10 ms | BodyMotion和姿态环周期 |
| 静止校准样本 | 100次 | 角速度零偏样本数 |
| 角速度低通alpha | 512 Q1024 | 0.5 |
| yaw角度校正beta | 512 Q1024 | 0.5 |
| 姿态预测时间 | 10 ms | 前向预测 |
| JY61超时 | 100 ms | 超时后关闭前馈 |
| JY61方向符号 | 1 | 保持坐标方向 |
| JY61前馈Kff | 1024 Q1024 | 1.0 |

### H7反馈与yaw执行器

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| H7反馈超时 | 100 ms | 姿态帧过期后停yaw |
| H7反馈方向符号 | -1 | 反转H7 yaw坐标方向 |
| H7角度死区 | 100 | 1.00度 |
| H7角度Kp | 8192 Q1024 | 8.0 |
| H7角速度Kp | 100 Q1024 | 约0.0977 |
| yaw一圈STEP数 | 3200 | 机械一圈 |
| 电机方向符号 | -1 | 正yaw到DIR的映射 |
| 最终yaw速度上限 | 1600 step/s | Task4/Task8限幅 |
| 姿态模式斜坡步长 | 200 step/s/次 | 每1ms更新 |
| 相对位置限位 | 25600 step | 相对姿态辅助启动位置 |

## 主要引脚快照

| 功能 | 当前引脚 |
| --- | --- |
| 左轮PWM / IN1 / IN2 | PA12 / PA21 / PA23 |
| 左轮编码器A / B | PA13 / PB24 |
| 右轮PWM / IN1 / IN2 | PA22 / PA31 / PA28 |
| 右轮编码器A / B | PB19 / PB20 |
| yaw STEP / DIR | PA8 / PA9 |
| pitch STEP / DIR | PA7 / PB18 |
| 数字灰度S1至S7 | PA15、PA16、PA17、PA24、PA25、PA26、PA27 |
| 本地OLED SDA / SCL | PA0 / PA1，I2C0 100 kHz |
| H7 LCD TX / 姿态RX | PA10 / PA11，UART0 |
| K230视觉 TX / RX | PB2 / PB3，UART3 115200 |
| JY61 TX / RX | PB6 / PB7，UART1 |
| K1 / K2 | PB9 / PB8，低有效 |
| 状态灯 | PA14 |

## 当前关闭或未标定方法

以下方法不属于当前生效参数，不应按其占位0值进行实车判断：

- 单轮锁死直角转向：代码和独立参数仍保留，但当前未选择。
- 直接PWM循迹：参数仍保留，但当前选择编码器速度闭环。
- PWM加编码器交叉同步循迹：参数仍保留，但当前未选择。
- 云台丢目标yaw扫描：当前关闭，速度和幅度仍为0。
- Task4近/中/远固定yaw随动：当前关闭，三段STEP数仍为0。
- 模拟ADC灰度：保留编译能力，当前使用数字GPIO灰度。

## 核对方式

本快照已根据TI Arm Clang预处理后的当前宏分支核对。后续修改参数后，可以执行：

```powershell
Set-Location D:\Ti\m0-light-rtos
.\tools\build_ccs.ps1 -Clean
```

构建只验证编译和链接，不会自动下载到板子。实车标定值仍应按架空轮、低限幅、
单任务、完整赛道的顺序验证。
