#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

/*
 * CAR_RECOVERY_SAFE_BUILD：芯片恢复用安全构建开关。
 * 1：只跑 PA14 心跳和恢复串口；0：正常比赛固件。
 */
#define CAR_RECOVERY_SAFE_BUILD        (0U)

/*
 * 日志策略：
 * - 启动初始化阶段允许少量日志。
 * - 主循环、按键、菜单、视觉坏帧、NO YAW 运行过程不打印。
 */
#define CAR_ENABLE_LOG_UART            (1U)

/* UART0 RX/PA11 由 H7 姿态链路独占；日志只保留 PA10 TX。 */
#define CAR_ENABLE_LOG_UART_RX         (0U)

/* PA14 状态灯：慢闪表示主循环存活，常亮表示致命错误。 */
#define CAR_ENABLE_PA14_DEBUG_LED      (1U)

/*
 * UI任务和硬件看门狗。
 *
 * ENABLE：1 创建监督任务并启动WWDT0；0 不创建任务，也不启动WWDT0。
 * TASK_PRIORITY：FreeRTOS优先级，当前范围0~6；默认5，低于底盘/云台控制6。
 * TASK_STACK_WORDS：栈深度单位是32位word；96 word等于384字节。
 * CHECK_PERIOD_MS：监督任务读取一次UI心跳的周期。
 * UI_TIMEOUT_MS：UI心跳多久不变化后停止喂狗；内部按CHECK周期向上取整。
 *
 * WWDT0使用32768Hz LFCLK，硬件超时公式为：
 * divider * 2^period_bits / 32768 秒。
 * 默认DIVIDE_8 + 12_BITS约1秒；10_BITS约250ms；15_BITS约8秒。
 * 最坏复位时间约为UI_TIMEOUT_MS向上取整后的时间，再加一个WWDT硬件周期。
 *
 * 注意：MSPM0G3507的WWDT0违规触发SYSRST，不是整板断电，也不会切断
 * 四线OLED的VCC。OLED控制器自身锁死时，MCU复位后屏幕仍可能不亮。
 */
#define CAR_ENABLE_UI_WATCHDOG         (0U)  /* 1：启用UI监督任务和WWDT0；0：全部关闭。 */
#define CAR_WATCHDOG_TASK_PRIORITY     (5U)  /* 监督任务优先级，低于优先级6的控制任务。 */
#define CAR_WATCHDOG_TASK_STACK_WORDS  (96U) /* 任务栈深度，96个32位word即384字节。 */
#define CAR_WATCHDOG_CHECK_PERIOD_MS   (250U) /* 每250ms检查一次UI心跳。 */
#define CAR_WATCHDOG_UI_TIMEOUT_MS     (2000U) /* UI连续2秒无心跳后停止喂狗。 */
#define CAR_WATCHDOG_HW_CLOCK_DIVIDER  DL_WWDT_CLOCK_DIVIDE_8 /* LFCLK进行8分频。 */
#define CAR_WATCHDOG_HW_TIMER_PERIOD   DL_WWDT_TIMER_PERIOD_12_BITS /* 2^12计数，当前约1秒。 */

/*
 * 两路云台步进轴公共参数。
 * 速度单位为 step/s；TIMG6 每 50us 调度一次 STEP，方向由 DIR 引脚单独控制。
 */
#define CAR_STEPPER_SPEED_MAX_SPS      (1600U) /* yaw/pitch 目标速度绝对值上限。 */
#define CAR_STEPPER_PULSE_HIGH_TICKS   (1U)     /* STEP 高电平保持 1 个 TIMG6 tick，即 50us。 */
#define CAR_STEPPER_RAMP_PERIOD_MS     (1U)     /* 每 1ms 更新一次当前 STEP 速度。 */
#define CAR_STEPPER_ACCEL_STEP_SPS     (80U)    /* 每 1ms 最多增加 80 step/s。 */
#define CAR_STEPPER_DECEL_STEP_SPS     (80U)    /* 每 1ms 最多减少 80 step/s。 */

/* Task3/Task4 起步搜点共用的 yaw 固定搜索速度。 */
#define CAR_GIMBAL_SEARCH_YAW_SPEED_SPS (400U)

#if (CAR_GIMBAL_SEARCH_YAW_SPEED_SPS > CAR_STEPPER_SPEED_MAX_SPS)
#error "CAR_GIMBAL_SEARCH_YAW_SPEED_SPS exceeds stepper speed limit"
#endif

/* 云台 EN 无可用 MCU 引脚；该值只记录状态机默认请求，硬件必须固定有效。 */
#define CAR_GIMBAL_ENABLE_DEFAULT_ON   (0U)     /* 0：上电默认不启用云台闭环；1：默认请求启用。 */

/* 底盘左右轮安装方向修正。 */
#define CAR_CHASSIS_LEFT_REVERSE       (0U)
#define CAR_CHASSIS_RIGHT_REVERSE      (1U)

/* K1/K2 按键：低电平按下，正式版不保留按键诊断日志。 */
#define CAR_KEY_ACTIVE_LOW             (1U)

/*
 * 云台闭环参数。
 * pitch 没有回零开关，软限位以每次启用闭环时的位置作为相对 0 度。
 */
#define CAR_GIMBAL_YAW_REVERSE         (0U)  /* 1：反转 yaw 控制方向；0：保持计算方向。 */
#define CAR_GIMBAL_PITCH_REVERSE       (0U)  /* 1：反转 pitch 控制方向；0：保持计算方向。 */
#define CAR_GIMBAL_VISION_TIMEOUT_TICKS (60U) /* 视觉数据 60ms 未更新时停止视觉追点。 */
#define CAR_GIMBAL_PITCH_LIMIT_STEPS   (600U) /* pitch 相对启用位置允许正负 200 STEP。 */

/*
 * ---------- Task1/Task4 普通循迹算法总开关 ----------
 * 0：编译第3/4套直接PWM配置；1：编译第1/2套目标速度PID配置。
 * 该开关同时选择Task1和Task4的算法，但两个任务始终读取各自独立的参数。
 * 这里只切普通循迹；直角强转和出弯始终使用下方的目标速度闭环参数。
 */
#define CAR_MOTOR_NO_YAW_USE_SPEED_PID         (1U)

#if (CAR_MOTOR_NO_YAW_USE_SPEED_PID > 1U)
#error "CAR_MOTOR_NO_YAW_USE_SPEED_PID must be 0 or 1"
#endif

/*
 * ---------- 直接PWM左右编码速度交叉同步总开关 ----------
 * 0：关闭左右速度交叉，只保留基准PWM和灰度差速。
 * 1：允许左右速度交叉，但仍只有S4压线、两轮非零同向时才实际生效。
 * 该开关只影响直接PWM方案，不影响目标速度PID和直角强转闭环。
 */
#define CAR_MOTOR_NO_YAW_ENABLE_CROSS_SYNC      (1U)

#if (CAR_MOTOR_NO_YAW_ENABLE_CROSS_SYNC > 1U)
#error "CAR_MOTOR_NO_YAW_ENABLE_CROSS_SYNC must be 0 or 1"
#endif

/*
 * S1到S7按车头朝前时从左到右排列，S4为中心。
 * 多个传感器同时命中时先求权重平均值，再乘100得到lineError。
 * 权重为正会降低左轮、提高右轮，车体左转；负值方向相反。
 * 两种算法、两个任务分别保存参数，禁止 Task4 再别名 Task1。
 */
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID

/*
 * ---------- 第1套：Task1 灰度目标速度 + PID ----------
 * 速度单位均为encoder count/20ms，与Task5 target命令相同。
 * adjustedError = deadband(lineError)
 * correction = (adjustedError * P_GAIN + errorDelta * D_GAIN) / 100
 * leftTarget = clamp(BASE - correction, MIN, MAX)
 * rightTarget = clamp(BASE + correction, MIN, MAX)
 * 增大P会加大当前偏差修正；增大D会加强对偏差突变的抑制，也更容易放大毛刺。
 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_BASE_SPEED_COUNTS_PER_PERIOD       (30)
#define CAR_MOTOR_NO_YAW_TASK1_PID_MIN_SPEED_COUNTS_PER_PERIOD         (-5)
#define CAR_MOTOR_NO_YAW_TASK1_PID_MAX_SPEED_COUNTS_PER_PERIOD        (35)
#define CAR_MOTOR_NO_YAW_TASK1_PID_LINE_DEADBAND                       (1) /* lineError单位，内部已乘100。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_GRAY_P_GAIN                         (3)
#define CAR_MOTOR_NO_YAW_TASK1_PID_GRAY_D_GAIN                         (0)
#define CAR_MOTOR_NO_YAW_TASK1_PID_CORRECTION_LIMIT_COUNTS            (30U) /* 单侧最大差速修正。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_SEARCH_SPEED_COUNTS_PER_PERIOD     (10) /* 启动即丢线且无上一拍时左轮搜线。 */
/* S1/S7是最外侧，绝对权重最大；S4居中时不产生差速。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_S1_WEIGHT                          (13)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S2_WEIGHT                           (11)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S3_WEIGHT                           (5)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S4_WEIGHT                           (0)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S5_WEIGHT                          (-5)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S6_WEIGHT                          (-11)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S7_WEIGHT                         (-13)

/*
 * ---------- 第2套：Task4 灰度目标速度 + PID ----------
 * 字段、单位和计算公式与第1套完全相同，但数值独立，修改这里不会影响Task1。
 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_BASE_SPEED_COUNTS_PER_PERIOD       (20)
#define CAR_MOTOR_NO_YAW_TASK4_PID_MIN_SPEED_COUNTS_PER_PERIOD         (-5)
#define CAR_MOTOR_NO_YAW_TASK4_PID_MAX_SPEED_COUNTS_PER_PERIOD        (35)
#define CAR_MOTOR_NO_YAW_TASK4_PID_LINE_DEADBAND                       (1)
#define CAR_MOTOR_NO_YAW_TASK4_PID_GRAY_P_GAIN                         (3)
#define CAR_MOTOR_NO_YAW_TASK4_PID_GRAY_D_GAIN                         (0)
#define CAR_MOTOR_NO_YAW_TASK4_PID_CORRECTION_LIMIT_COUNTS            (30U)
#define CAR_MOTOR_NO_YAW_TASK4_PID_SEARCH_SPEED_COUNTS_PER_PERIOD     (10)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S1_WEIGHT                          (13)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S2_WEIGHT                           (11)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S3_WEIGHT                           (3)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S4_WEIGHT                           (0)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S5_WEIGHT                          (-3)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S6_WEIGHT                          (-11)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S7_WEIGHT                         (-13)

#else

/*
 * ---------- 第3套：Task1 直接PWM + 灰度 + 编码器交叉同步 ----------
 * PWM单位为raw timer count；当前软件上限1200，因此BASE=300等于25%。
 * grayCorrection = deadband(lineError) * GRAY_GAIN / 100
 * leftCommand = BASE - grayCorrection；rightCommand = BASE + grayCorrection
 * syncCorrection = (leftFeedback - rightFeedback) * SYNC_GAIN_Q1024 / 1024
 * leftOutput = leftCommand - syncCorrection；rightOutput = rightCommand + syncCorrection
 * SYNC只在S4确认压线、两轮命令同向且都非0时生效。S4离线时同步增益强制为0，
 * 只保留灰度PWM差速。调大GRAY增强转向；调大SYNC增强居中直行时的同速能力，
 * 但SYNC过大仍可能表现为左右PWM来回抢。
 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_BASE_COUNTS                       (240)
#define CAR_MOTOR_NO_YAW_TASK1_PWM_SEARCH_COUNTS                     (120) /* 启动即丢线且无上一拍时左轮PWM。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_LINE_DEADBAND                       (0) /* lineError单位，内部已乘100。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_GRAY_GAIN                           (2)
#define CAR_MOTOR_NO_YAW_TASK1_PWM_GRAY_LIMIT_COUNTS                 (120U) /* 单侧灰度PWM修正上限。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_SYNC_GAIN_Q1024                  (3072L) /* 3072=每差1速度count修正3 PWM count。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_SYNC_LIMIT_COUNTS                 (120U) /* 单侧同步PWM修正上限。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S1_WEIGHT                          (11)
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S2_WEIGHT                           (5)
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S3_WEIGHT                           (3)
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S4_WEIGHT                           (0)
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S5_WEIGHT                          (-3)
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S6_WEIGHT                          (-5)
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S7_WEIGHT                         (-11)

/*
 * ---------- 第4套：Task4 直接PWM + 灰度 + 编码器交叉同步 ----------
 * 字段、单位和计算公式与第3套完全相同，但数值独立，修改这里不会影响Task1。
 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_BASE_COUNTS                       (240)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_SEARCH_COUNTS                     (120)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_LINE_DEADBAND                       (0)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_GRAY_GAIN                           (1)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_GRAY_LIMIT_COUNTS                 (120U)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_SYNC_GAIN_Q1024                  (3072L)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_SYNC_LIMIT_COUNTS                 (120U)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S1_WEIGHT                          (11)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S2_WEIGHT                           (5)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S3_WEIGHT                           (3)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S4_WEIGHT                           (0)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S5_WEIGHT                          (-3)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S6_WEIGHT                          (-5)
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S7_WEIGHT                         (-11)

#endif

/*
 * ---------- Task1 强转和出弯参数 ----------
 * 所有速度仍为count/20ms；负数表示反转。左转时左轮是内轮、右轮是外轮，
 * 右转时相反。TURN_SPEED是刚入强转的外轮速度，TURN_NEAR_SPEED是接近
 * TURN_TARGET_ANGLE时的外轮速度；两者之间按JY61已转角线性插值。
 * TURN_MIN_ENCODER_GAP是两次强转间允许的最小平均累计编码距离。
 * TURN_APPROACH是识别直角后继续沿上一拍命令前进的时间。
 * TURN_EXIT的INNER/OUTER是回线后的短暂出弯速度；HOLD是找不到回线的安全超时。
 * LINE_LOST_TIMEOUT=0表示普通循迹丢线后不自动停车。
 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_INNER_SPEED_COUNTS_PER_PERIOD     (-1)
#define CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_SPEED_COUNTS_PER_PERIOD       (25)   //入弯道
#define CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD  (25)   //出弯道
#define CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD      (25)
#define CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD (25)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_MIN_ENCODER_GAP_COUNTS          (3500U)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_APPROACH_MS                       (100U)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD (25)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD (26)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_MS                           (40U)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_HOLD_MS                         (10000U)
#define CAR_MOTOR_NO_YAW_TASK1_LINE_LOST_TIMEOUT_MS                    (0U)

/* Task4强转/出弯字段含义与Task1相同，但全部保存为独立数值。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_INNER_SPEED_COUNTS_PER_PERIOD     (-1)
#define CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_SPEED_COUNTS_PER_PERIOD       (30)
#define CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD  (30)
#define CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD      (30)
#define CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD (30)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_MIN_ENCODER_GAP_COUNTS          (3500U)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_APPROACH_MS                       (200U)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD (25)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD (26)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_MS                           (40U)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_HOLD_MS                         (5000U)
#define CAR_MOTOR_NO_YAW_TASK4_LINE_LOST_TIMEOUT_MS                    (0U)

/*
 * 两任务共用的灰度采样语义；这些值改变ISR判定时序，不属于四套调速参数。
 * 左/右窗口表示S1+S2或S6+S7在该时间内都出现才判直角。
 * LINE_CONFIRM要求同一完整mask连续出现指定次数才更新普通循迹输入。
 * TURN_REARM要求外侧传感器稳定释放后才允许识别下一次直角。
 * RETURN_CONFIRM要求转向侧最外传感器重新命中指定次数才进入出弯。
 */
#define CAR_MOTOR_NO_YAW_LEFT_TURN_WINDOW_MS   (100U)  /* 左侧S1/S2组合窗口。 */
#define CAR_MOTOR_NO_YAW_RIGHT_TURN_WINDOW_MS  (100U)  /* 右侧S6/S7组合窗口。 */
#define CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US       (1000U) /* 灰度语义采样间隔，当前1ms。 */
#define CAR_MOTOR_NO_YAW_LINE_CONFIRM_SAMPLES  (2U)    /* 普通mask连续确认次数。 */
#define CAR_MOTOR_NO_YAW_TURN_REARM_MS         (20U)   /* 下一次直角检测重新开门时间。 */
#define CAR_MOTOR_NO_YAW_RETURN_CONFIRM_SAMPLES (2U)   /* 最外传感器回线确认次数。 */
#define CAR_MOTOR_NO_YAW_TURN_TARGET_ANGLE_X100 (7500U) /* JY61外轮减速参考角75.00度。 */

/*
 * Task4 云台 yaw 随动参数。
 * NEAR/MID/FAR三组只控制Task4强转yaw，按近/中/远/中循环；视觉KP不再切表。
 * BASE_SPEED_SPS：循迹期间固定的 yaw 基础速度；负数可反向。
 * STEPS：每次强转开始后，yaw 额外跟随的 STEP 数，方向由速度符号决定。
 * SPEED_SPS：强转期间单独使用的 yaw 目标速度。
 * ACCEL/DECEL_STEP_SPS：强转期间只给 yaw 轴使用的更快斜坡；DECEL 是降速斜坡。
 * LOST_SEARCH：视觉超时后以丢失位置为中心左右摆动，重新收到帧后立即退出。
 */
#define CAR_MISSION4_GIMBAL_YAW_BASE_SPEED_SPS       (0)   //一圈3200
#define CAR_MISSION4_GIMBAL_LOST_SEARCH_SPEED_SPS    (0U)
#define CAR_MISSION4_GIMBAL_LOST_SEARCH_AMPLITUDE_STEPS (0U)
#define CAR_MISSION4_LASER_TO_LINE_DELAY_MS          (500U) /* 首帧发F后等待多久再启动底盘循迹。 */
#define CAR_MISSION4_GIMBAL_CORRECTION_RELEASE_DELAY_MS (80U) /* 灰度确认回线后继续姿态矫正并屏蔽视觉的时间。 */
#define CAR_MISSION4_EXTRA_ENCODER_COUNTS             (3500U)  //任务四编码值

#define CAR_MISSION4_GIMBAL_NEAR_YAW_STEPS          (0U)  //强转步数
#define CAR_MISSION4_GIMBAL_NEAR_YAW_SPEED_SPS      (400)
#define CAR_MISSION4_GIMBAL_NEAR_YAW_ACCEL_STEP_SPS (80U)  //加速度
#define CAR_MISSION4_GIMBAL_NEAR_YAW_DECEL_STEP_SPS (80U)

#define CAR_MISSION4_GIMBAL_MID_YAW_STEPS           (0U)
#define CAR_MISSION4_GIMBAL_MID_YAW_SPEED_SPS       (400U)
#define CAR_MISSION4_GIMBAL_MID_YAW_ACCEL_STEP_SPS  (80U)
#define CAR_MISSION4_GIMBAL_MID_YAW_DECEL_STEP_SPS  (80U)

#define CAR_MISSION4_GIMBAL_FAR_YAW_STEPS           (0U)
#define CAR_MISSION4_GIMBAL_FAR_YAW_SPEED_SPS       (400U)
#define CAR_MISSION4_GIMBAL_FAR_YAW_ACCEL_STEP_SPS  (80U)
#define CAR_MISSION4_GIMBAL_FAR_YAW_DECEL_STEP_SPS  (80U)

/* 主循环和 OLED 菜单刷新节拍。 */
#define CAR_APP_LOOP_DELAY_MS          (1U)
#define CAR_MENU_REFRESH_MS            (100U)
#define CAR_MENU_REFRESH_TICKS \
    ((CAR_MENU_REFRESH_MS + CAR_APP_LOOP_DELAY_MS - 1U) / \
        CAR_APP_LOOP_DELAY_MS)

/*
 * 灰度输入。
 * 数字量模式下 Gray_ReadDigitalMaskFast 直接读 GPIO，NO YAW 定时器中断使用这条快路径。
 */
#define CAR_GRAY_INPUT_DIGITAL         (1U)
#define GRAY_DIGITAL_ACTIVE_HIGH       (1U)
#define GRAY_DIGITAL_INPUT_PULL_UP     (1U)
#define GRAY_SENSOR_COUNT              (7U)
#define CAR_GRAY_TRACK_SENSOR_MASK     (0x7FU)

/* 模拟模式保留编译能力，正式车当前不走 ADC 灰度。 */
#define GRAY_ACTIVE_HIGH               (1U)
#define GRAY_ADC_MAX_VALUE             (4095U)
#define GRAY_DEFAULT_THRESHOLD         (2000U)
#define GRAY_ADC_TIMEOUT_COUNT         (100000U)
#define GRAY_FILTER_SAMPLE_COUNT       (1U)
#define GRAY_DIGITAL_CONFIRM_COUNT     (1U)
#define GRAY_LINE_ERROR_SCALE          (100)

#endif
