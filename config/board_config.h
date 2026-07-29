#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "gmr_board_config.h"

#else

/*
 * CAR_RECOVERY_SAFE_BUILD：芯片恢复用安全构建开关。
 * 1：只跑板载状态灯心跳和恢复串口；0：正常比赛固件。
 */
#define CAR_RECOVERY_SAFE_BUILD        (0U)

/*
 * 日志策略：
 * - 启动初始化阶段允许少量日志。
 * - 主循环、按键、菜单、视觉坏帧、NO YAW 运行过程不打印。
 */
#ifndef CAR_ENABLE_LOG_UART
#define CAR_ENABLE_LOG_UART            (1U)
#endif

/* UART0 RX/PA11 由 H7 姿态链路独占；日志只保留 PA10 TX。 */
#ifndef CAR_ENABLE_LOG_UART_RX
#define CAR_ENABLE_LOG_UART_RX         (0U)
#endif

/*
 * UART0硬件需求由库组合自动推导，不要手动修改：
 * - 日志或H7 LCD需要PA10 TX。
 * - 双IMU姿态矫正需要PA11 RX。
 * 即使关闭普通日志，只要H7 LCD开启，UART0发送仍会正常初始化。
 */
#define CAR_UART0_TX_REQUIRED \
    ((CAR_ENABLE_LOG_UART != 0U) || CAR_LIBRARY_H7_LCD_ENABLED)
#define CAR_UART0_REQUIRED \
    (CAR_UART0_TX_REQUIRED || CAR_LIBRARY_H7_IMU_ENABLED)

/* PA11不能同时解析H7二进制姿态帧和普通文本日志。 */
#if (CAR_LIBRARY_H7_IMU_ENABLED && CAR_ENABLE_LOG_UART_RX)
#error "H7 attitude feedback and text log RX cannot share UART0 RX"
#endif

/*
 * 本地SSD1306 OLED的具体显示参数；只有选择OLED方法后CCS才高亮本段。
 * 12像素字体可显示5行，每行安全显示20个ASCII字符，适合现有四行菜单。
 * OLED使用同步I2C整屏刷新，只能在低优先级UI任务使用，禁止放进控制ISR。
 */
#if CAR_LIBRARY_LOCAL_OLED_ENABLED
#define CAR_LOCAL_OLED_FONT_SIZE_PIXELS        (12U) /* 支持12、16或24像素ASCII字模。 */
#define CAR_LOCAL_OLED_VISIBLE_ROWS \
    (64U / CAR_LOCAL_OLED_FONT_SIZE_PIXELS)          /* 由64像素屏高自动计算。 */
#define CAR_LOCAL_OLED_MAX_CHARS_PER_ROW \
    ((128U / (CAR_LOCAL_OLED_FONT_SIZE_PIXELS / 2U)) - 1U) /* 防止最后一字自动换行。 */

#if ((CAR_LOCAL_OLED_FONT_SIZE_PIXELS != 12U) && \
    (CAR_LOCAL_OLED_FONT_SIZE_PIXELS != 16U) && \
    (CAR_LOCAL_OLED_FONT_SIZE_PIXELS != 24U))
#error "Local OLED font size must be 12, 16 or 24 pixels"
#endif
#endif

/* 板载状态灯：地猛星为PA14，天猛星为PB22。 */
#define CAR_ENABLE_DEBUG_LED           (1U)
/* 兼容旧名；新代码不再把状态灯绑定到PA14。 */
#define CAR_ENABLE_PA14_DEBUG_LED      CAR_ENABLE_DEBUG_LED

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

/* Task3/Task4 起步搜点；未选择固定yaw搜索时保留0作为内部停转值。 */
#if CAR_LIBRARY_GIMBAL_STARTUP_SEARCH_ENABLED
#define CAR_GIMBAL_SEARCH_YAW_SPEED_SPS (400U) /* 固定yaw搜点速度，单位step/s；越大搜索越快。 */
#else
#define CAR_GIMBAL_SEARCH_YAW_SPEED_SPS (0U)   /* 搜点库关闭后的内部停转占位，不要调。 */
#endif

#if CAR_LIBRARY_GIMBAL_STARTUP_SEARCH_ENABLED
#if (CAR_GIMBAL_SEARCH_YAW_SPEED_SPS > CAR_STEPPER_SPEED_MAX_SPS)
#error "CAR_GIMBAL_SEARCH_YAW_SPEED_SPS exceeds stepper speed limit"
#endif
#endif

/* 云台 EN 无可用 MCU 引脚；该值只记录状态机默认请求，硬件必须固定有效。 */
#define CAR_GIMBAL_ENABLE_DEFAULT_ON   (0U)     /* 0：上电默认不启用云台闭环；1：默认请求启用。 */

/* 底盘左右轮安装方向修正。 */
#define CAR_CHASSIS_LEFT_REVERSE       (0U) /* 1反转左轮命令方向；0保持驱动接线方向。 */
#define CAR_CHASSIS_RIGHT_REVERSE      (1U) /* 1反转右轮命令方向；当前实车右轮需反转。 */

/* K1/K2 按键：低电平按下，正式版不保留按键诊断日志。 */
#define CAR_KEY_ACTIVE_LOW             (1U) /* 1表示按下为低电平；接线改变后才允许修改。 */

/*
 * 云台闭环参数。
 * pitch 没有回零开关，软限位以每次启用闭环时的位置作为相对 0 度。
 */
#define CAR_GIMBAL_YAW_REVERSE         (0U)  /* 1：反转 yaw 控制方向；0：保持计算方向。 */
#define CAR_GIMBAL_PITCH_REVERSE       (0U)  /* 1：反转 pitch 控制方向；0：保持计算方向。 */
#define CAR_GIMBAL_VISION_TIMEOUT_TICKS (60U) /* 视觉数据 60ms 未更新时停止视觉追点。 */
#define CAR_GIMBAL_PITCH_LIMIT_STEPS   (600U) /* pitch相对启用位置允许正负600 STEP。 */

/*
 * S1到S7按车头朝前时从左到右排列，S4为中心。
 * 多个传感器同时命中时先求权重平均值，再乘100得到lineError。
 * 权重为正会降低左轮、提高右轮，车体左转；负值方向相反。
 * 两种算法、两个任务分别保存参数，禁止 Task4 再别名 Task1。
 */
#if CAR_LIBRARY_LINE_DRIVE_USES_SPEED_LOOP

/*
 * ---------- 第1套：Task1 灰度目标速度 + PID ----------
 * 速度单位均为encoder count/20ms，与Task5 target命令相同。
 * adjustedError = deadband(lineError)
 * correction = (adjustedError * P_GAIN + errorDelta * D_GAIN) / 100
 * leftTarget = clamp(BASE - correction, MIN, MAX)
 * rightTarget = clamp(BASE + correction, MIN, MAX)
 * 增大P会加大当前偏差修正；增大D会加强对偏差突变的抑制，也更容易放大毛刺。
 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_BASE_SPEED_COUNTS_PER_PERIOD       (30) /* 居中循迹时两轮基础目标速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_MIN_SPEED_COUNTS_PER_PERIOD         (-5) /* 灰度修正后单轮允许的最低目标速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_MAX_SPEED_COUNTS_PER_PERIOD        (35) /* 灰度修正后单轮允许的最高目标速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_LINE_DEADBAND                       (1) /* lineError死区；内部误差已经乘100。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_GRAY_P_GAIN                         (3) /* 当前灰度偏差增益；调大后转向更积极。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_GRAY_D_GAIN                         (0) /* 灰度偏差变化率增益；调大会抑制突变也会放大毛刺。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_CORRECTION_LIMIT_COUNTS            (30U) /* 单侧最大差速修正。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_SEARCH_SPEED_COUNTS_PER_PERIOD     (10) /* 启动即丢线且无上一拍时左轮搜线。 */
/* S1/S7是最外侧，绝对权重最大；S4居中时不产生差速。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_S1_WEIGHT                          (13) /* 最左S1命中时的正向左转权重。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_S2_WEIGHT                           (11) /* 左侧S2命中时的较大左转权重。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_S3_WEIGHT                           (5) /* 左侧S3命中时的小幅左转权重。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_S4_WEIGHT                           (0) /* 中心S4命中时不做左右差速。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_S5_WEIGHT                          (-5) /* 右侧S5命中时的小幅右转权重。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_S6_WEIGHT                          (-11) /* 右侧S6命中时的较大右转权重。 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_S7_WEIGHT                         (-13) /* 最右S7命中时的最大右转权重。 */

/*
 * ---------- 第2套：Task4 灰度目标速度 + PID ----------
 * 字段、单位和计算公式与第1套完全相同，但数值独立，修改这里不会影响Task1。
 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_BASE_SPEED_COUNTS_PER_PERIOD       (20) /* Task4居中循迹时两轮基础目标速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_MIN_SPEED_COUNTS_PER_PERIOD         (-5) /* Task4单轮最低目标速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_MAX_SPEED_COUNTS_PER_PERIOD        (35) /* Task4单轮最高目标速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_LINE_DEADBAND                       (1) /* Task4灰度误差死区，单位lineError。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_GRAY_P_GAIN                         (3) /* Task4灰度比例增益。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_GRAY_D_GAIN                         (0) /* Task4灰度变化率增益；0表示关闭D项。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_CORRECTION_LIMIT_COUNTS            (30U) /* Task4单轮差速修正绝对值上限。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_SEARCH_SPEED_COUNTS_PER_PERIOD     (10) /* Task4启动即丢线时的默认搜线速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_S1_WEIGHT                          (13) /* Task4最左S1权重。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_S2_WEIGHT                           (11) /* Task4左侧S2权重。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_S3_WEIGHT                           (3) /* Task4左侧S3权重。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_S4_WEIGHT                           (0) /* Task4中心S4权重。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_S5_WEIGHT                          (-3) /* Task4右侧S5权重。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_S6_WEIGHT                          (-11) /* Task4右侧S6权重。 */
#define CAR_MOTOR_NO_YAW_TASK4_PID_S7_WEIGHT                         (-13) /* Task4最右S7权重。 */

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
#define CAR_MOTOR_NO_YAW_TASK1_PWM_BASE_COUNTS                       (240) /* Task1居中循迹基础PWM，单位timer count。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_SEARCH_COUNTS                     (120) /* 启动即丢线且无上一拍时左轮PWM。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_LINE_DEADBAND                       (0) /* lineError单位，内部已乘100。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_GRAY_GAIN                           (2) /* 灰度误差转成PWM差速的增益。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_GRAY_LIMIT_COUNTS                 (120U) /* 单侧灰度PWM修正上限。 */
#if CAR_LIBRARY_LINE_DRIVE_USES_PWM_ENCODER_SYNC
#define CAR_MOTOR_NO_YAW_TASK1_PWM_SYNC_GAIN_Q1024                  (3072L) /* 3072=每差1速度count修正3 PWM count。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_SYNC_LIMIT_COUNTS                 (120U) /* 单侧同步PWM修正上限。 */
#else
#define CAR_MOTOR_NO_YAW_TASK1_PWM_SYNC_GAIN_Q1024                     (0L) /* 未选同步方法时的内部禁用值。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_SYNC_LIMIT_COUNTS                    (0U) /* 未选同步方法时不允许同步修正。 */
#endif
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S1_WEIGHT                          (11) /* Task1 PWM方法最左S1权重。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S2_WEIGHT                           (5) /* Task1 PWM方法左侧S2权重。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S3_WEIGHT                           (3) /* Task1 PWM方法左侧S3权重。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S4_WEIGHT                           (0) /* Task1 PWM方法中心S4权重。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S5_WEIGHT                          (-3) /* Task1 PWM方法右侧S5权重。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S6_WEIGHT                          (-5) /* Task1 PWM方法右侧S6权重。 */
#define CAR_MOTOR_NO_YAW_TASK1_PWM_S7_WEIGHT                         (-11) /* Task1 PWM方法最右S7权重。 */

/*
 * ---------- 第4套：Task4 直接PWM + 灰度 + 编码器交叉同步 ----------
 * 字段、单位和计算公式与第3套完全相同，但数值独立，修改这里不会影响Task1。
 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_BASE_COUNTS                       (240) /* Task4居中循迹基础PWM。 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_SEARCH_COUNTS                     (120) /* Task4启动即丢线时的默认搜线PWM。 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_LINE_DEADBAND                       (0) /* Task4 PWM灰度误差死区。 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_GRAY_GAIN                           (1) /* Task4灰度误差转PWM差速的增益。 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_GRAY_LIMIT_COUNTS                 (120U) /* Task4单轮灰度PWM修正上限。 */
#if CAR_LIBRARY_LINE_DRIVE_USES_PWM_ENCODER_SYNC
#define CAR_MOTOR_NO_YAW_TASK4_PWM_SYNC_GAIN_Q1024                  (3072L) /* 每差1速度count修正3 PWM count。 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_SYNC_LIMIT_COUNTS                 (120U) /* Task4同步PWM单侧修正上限。 */
#else
#define CAR_MOTOR_NO_YAW_TASK4_PWM_SYNC_GAIN_Q1024                     (0L) /* 未选同步方法时的内部禁用值。 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_SYNC_LIMIT_COUNTS                    (0U) /* 未选同步方法时不允许同步修正。 */
#endif
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S1_WEIGHT                          (11) /* Task4 PWM方法最左S1权重。 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S2_WEIGHT                           (5) /* Task4 PWM方法左侧S2权重。 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S3_WEIGHT                           (3) /* Task4 PWM方法左侧S3权重。 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S4_WEIGHT                           (0) /* Task4 PWM方法中心S4权重。 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S5_WEIGHT                          (-3) /* Task4 PWM方法右侧S5权重。 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S6_WEIGHT                          (-5) /* Task4 PWM方法右侧S6权重。 */
#define CAR_MOTOR_NO_YAW_TASK4_PWM_S7_WEIGHT                         (-11) /* Task4 PWM方法最右S7权重。 */

#endif

/*
 * ---------- 直角转向方法参数 ----------
 * 选择库方法后，CCS只高亮对应分支。两套完整保留各自已调数值，后续换赛道
 * 可以分别调整，不会覆盖另一种方法。所有速度单位都是count/20ms。
 *
 * 参数名与执行阶段：
 * - TURN_INNER_SPEED：强转阶段内轮速度。双轮反转法必须为负数；锁内轮法接近0。
 * - LEFT/RIGHT_TURN_SPEED：左转/右转刚开始时的外轮速度。
 * - LEFT/RIGHT_TURN_NEAR_SPEED：接近JY61参考角时的外轮速度；固定速度法不使用。
 * - TURN_MIN_ENCODER_GAP：两次强转之间至少行驶的平均编码器距离，防止重复触发。
 * - TURN_APPROACH：确认直角后继续沿用上一拍循迹命令的时间，让转轴靠近弯心。
 * - TURN_EXIT_OUTER/INNER_SPEED：最外灰度回线后的前进速度；内轮略快可帮助摆正。
 * - TURN_EXIT_MS：以上出弯速度保持多久，结束后恢复普通循迹。
 * - TURN_HOLD_MS：强转等待回线的安全超时，不是计划转弯时长；超时会停车报错。
 * - LINE_LOST_TIMEOUT_MS：普通循迹全无灰度多久后停车；0表示永不因丢线自动停车。
 *
 * 正常调赛道时只改当前高亮分支。速度绝对值不得超过control_config.h中的
 * CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD，否则motor_no_yaw.c会在编译时报错。
 */
#if (CAR_LIBRARY_RIGHT_ANGLE_TURN_METHOD == \
    CAR_LIBRARY_RIGHT_ANGLE_TURN_LOCKED_INNER)

/* 单轮锁死：内轮近似0，必须等最外传感器先释放再重新回线。 */
/* Task1单轮锁死参数：只影响Task1圈数循迹，不会改Task4。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_INNER_SPEED_COUNTS_PER_PERIOD     (-1) /* 强转内轮近似锁死；负1用于克服机构回差。 */
#define CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_SPEED_COUNTS_PER_PERIOD       (25) /* 左转外轮初始速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD  (25) /* 左转接近JY61参考角时的外轮速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD      (25) /* 右转外轮初始速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD (25) /* 右转接近JY61参考角时的外轮速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_MIN_ENCODER_GAP_COUNTS          (3500U) /* 两次强转间最小平均编码器距离，单位count。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_APPROACH_MS                       (100U) /* 识别直角后继续向前100ms再强转。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD (25) /* 回线后外轮向前速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD (26) /* 回线后内轮向前速度，略大于外轮帮助摆正。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_MS                           (40U) /* 出弯速度保持40ms后恢复循迹。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_HOLD_MS                         (10000U) /* 强转最多等待回线10s，超时立即停车。 */
#define CAR_MOTOR_NO_YAW_TASK1_LINE_LOST_TIMEOUT_MS                    (0U) /* 普通循迹丢线停车时间；0表示持续搜线。 */

/* Task4单轮锁死参数：与Task1独立，用于追点/追圆后的循迹阶段。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_INNER_SPEED_COUNTS_PER_PERIOD     (-1) /* Task4强转内轮近似锁死。 */
#define CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_SPEED_COUNTS_PER_PERIOD       (30) /* Task4左转外轮初始速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD  (30) /* Task4左转接近参考角时外轮速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD      (30) /* Task4右转外轮初始速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD (30) /* Task4右转接近参考角时外轮速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_MIN_ENCODER_GAP_COUNTS          (3500U) /* Task4两次强转间最小平均距离。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_APPROACH_MS                       (200U) /* Task4识别直角后继续向前200ms。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD (25) /* Task4回线后外轮向前速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD (26) /* Task4回线后内轮略快向前。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_MS                           (40U) /* Task4出弯速度保持时间。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_HOLD_MS                         (5000U) /* Task4强转等待回线最多5s。 */
#define CAR_MOTOR_NO_YAW_TASK4_LINE_LOST_TIMEOUT_MS                    (0U) /* Task4普通循迹丢线持续搜线。 */

#elif (CAR_LIBRARY_RIGHT_ANGLE_TURN_METHOD == \
    CAR_LIBRARY_RIGHT_ANGLE_TURN_COUNTER_ROTATE)

/* 双轮反转：内轮后退、外轮前进，最外传感器命中后直接进入出弯。 */
/* Task1双轮反转参数：左转看S1回线，右转看S7回线。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_INNER_SPEED_COUNTS_PER_PERIOD     (-25) /* 强转内轮后退速度；负号表示反向。 */
#define CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_SPEED_COUNTS_PER_PERIOD       (25) /* 左转时右侧外轮初始前进速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD  (25) /* 左转接近JY61参考角时右外轮速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD      (25) /* 右转时左侧外轮初始前进速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD (25) /* 右转接近JY61参考角时左外轮速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_MIN_ENCODER_GAP_COUNTS          (3500U) /* 两次强转至少间隔3500平均编码器count。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_APPROACH_MS                       (300U) /* 确认直角后保持原循迹命令前进200ms。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD (25) /* S1/S7回线后外轮同向前进速度。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD (26) /* S1/S7回线后内轮同向前进速度；略快帮助摆正。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_MS                           (40U) /* 出弯速度保持40ms后回到灰度循迹。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_HOLD_MS                         (10000U) /* 最长允许强转10s；未回线则安全停车。 */
#define CAR_MOTOR_NO_YAW_TASK1_LINE_LOST_TIMEOUT_MS                    (0U) /* 普通循迹全0时不超时停车，持续按旧方向搜线。 */

/* Task4双轮反转参数：独立于Task1，方便按Task4车速单独标定。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_INNER_SPEED_COUNTS_PER_PERIOD     (-30) /* Task4强转内轮后退速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_SPEED_COUNTS_PER_PERIOD       (30) /* Task4左转时右外轮初始前进速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD  (30) /* Task4左转接近参考角时右外轮速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD      (30) /* Task4右转时左外轮初始前进速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD (30) /* Task4右转接近参考角时左外轮速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_MIN_ENCODER_GAP_COUNTS          (3500U) /* Task4两次强转至少间隔3500平均count。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_APPROACH_MS                       (300U) /* Task4确认直角后继续向前200ms。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD (25) /* Task4回线后外轮同向前进速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD (26) /* Task4回线后内轮略快向前，帮助车身摆正。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_MS                           (40U) /* Task4出弯速度保持40ms。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_HOLD_MS                         (5000U) /* Task4最长强转5s，超时安全停车。 */
#define CAR_MOTOR_NO_YAW_TASK4_LINE_LOST_TIMEOUT_MS                    (0U) /* Task4普通循迹丢线时持续搜线。 */

#else

/* 未启用直角库时只提供内部编译占位，任务菜单不会开放Task1/Task4。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_INNER_SPEED_COUNTS_PER_PERIOD       (0)
#define CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_SPEED_COUNTS_PER_PERIOD        (0)
#define CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD   (0)
#define CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD       (0)
#define CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD  (0)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_MIN_ENCODER_GAP_COUNTS             (0U)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_APPROACH_MS                         (0U)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD  (0)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD  (0)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_MS                             (0U)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_HOLD_MS                            (10U)
#define CAR_MOTOR_NO_YAW_TASK1_LINE_LOST_TIMEOUT_MS                     (0U)

#define CAR_MOTOR_NO_YAW_TASK4_TURN_INNER_SPEED_COUNTS_PER_PERIOD       (0)
#define CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_SPEED_COUNTS_PER_PERIOD        (0)
#define CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD   (0)
#define CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD       (0)
#define CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD  (0)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_MIN_ENCODER_GAP_COUNTS             (0U)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_APPROACH_MS                         (0U)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD  (0)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD  (0)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_MS                             (0U)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_HOLD_MS                            (10U)
#define CAR_MOTOR_NO_YAW_TASK4_LINE_LOST_TIMEOUT_MS                     (0U)

#endif

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
#if CAR_LIBRARY_TURN_SPEED_USES_JY61
#define CAR_MOTOR_NO_YAW_TURN_TARGET_ANGLE_X100 (7500U) /* JY61外轮减速参考角75.00度。 */
#else
#define CAR_MOTOR_NO_YAW_TURN_TARGET_ANGLE_X100    (1U) /* 固定速度方法的内部占位。 */
#endif

/*
 * Task4 云台 yaw 随动参数。
 * NEAR/MID/FAR三组只控制Task4强转yaw，按近/中/远/中循环；视觉KP不再切表。
 * BASE_SPEED_SPS：循迹期间固定的 yaw 基础速度；负数可反向。
 * STEPS：每次强转开始后，yaw 额外跟随的 STEP 数，方向由速度符号决定。
 * SPEED_SPS：强转期间单独使用的 yaw 目标速度。
 * ACCEL/DECEL_STEP_SPS：强转期间只给 yaw 轴使用的更快斜坡；DECEL 是降速斜坡。
 * LOST_SEARCH：视觉超时后以丢失位置为中心左右摆动，重新收到帧后立即退出。
 */
#if CAR_LIBRARY_GIMBAL_TURN_FOLLOW_ENABLED
#define CAR_MISSION4_GIMBAL_YAW_BASE_SPEED_SPS       (0) /* Task4直线循迹时叠加的固定yaw速度；0不随动。 */
#else
#define CAR_MISSION4_GIMBAL_YAW_BASE_SPEED_SPS       (0) /* 随动库关闭后的内部停转值，不要调。 */
#endif

#if CAR_LIBRARY_GIMBAL_LOST_TARGET_ENABLED
/* 该分支框架已接入，但当前实车尚未标定速度和摆幅。 */
#define CAR_MISSION4_GIMBAL_LOST_SEARCH_SPEED_SPS    (0U) /* 丢目标时yaw扫描速度；0表示暂不转动。 */
#define CAR_MISSION4_GIMBAL_LOST_SEARCH_AMPLITUDE_STEPS (0U) /* 相对丢失位置的单侧扫描幅度。 */
#else
#define CAR_MISSION4_GIMBAL_LOST_SEARCH_SPEED_SPS    (0U) /* 丢目标恢复库关闭后的内部占位。 */
#define CAR_MISSION4_GIMBAL_LOST_SEARCH_AMPLITUDE_STEPS (0U) /* 丢目标恢复库关闭后的内部占位。 */
#endif
#define CAR_MISSION4_LASER_TO_LINE_DELAY_MS          (500U) /* 首帧发F后等待多久再启动底盘循迹。 */
#define CAR_MISSION4_GIMBAL_CORRECTION_RELEASE_DELAY_MS (80U) /* 灰度确认回线后继续姿态矫正并屏蔽视觉的时间。 */
#define CAR_MISSION4_GIMBAL_PITCH_EXIT_STEPS          (100U) /* 每次退出姿态矫正时pitch固定向上补偿的STEP数。 */
#define CAR_MISSION4_GIMBAL_PITCH_EXIT_SPEED_SPS      (400U) /* pitch退出补偿速度，方向固定为向上。 */
#define CAR_MISSION4_EXTRA_ENCODER_COUNTS             (3500U) /* Task4完成目标圈数后继续前进的平均编码器count。 */

#if CAR_LIBRARY_GIMBAL_TURN_FOLLOW_ENABLED
/* 分段固定yaw随动框架；当前步数为0，启用前必须完成实车标定。 */
#define CAR_MISSION4_GIMBAL_NEAR_YAW_STEPS          (0U) /* 近段每次强转需要额外跟随的yaw步数。 */
#define CAR_MISSION4_GIMBAL_NEAR_YAW_SPEED_SPS      (400) /* 近段强转yaw目标速度，符号决定方向。 */
#define CAR_MISSION4_GIMBAL_NEAR_YAW_ACCEL_STEP_SPS (80U) /* 近段每1ms最大加速量，单位step/s。 */
#define CAR_MISSION4_GIMBAL_NEAR_YAW_DECEL_STEP_SPS (80U) /* 近段每1ms最大减速量，单位step/s。 */

#define CAR_MISSION4_GIMBAL_MID_YAW_STEPS           (0U) /* 中段每次强转需要额外跟随的yaw步数。 */
#define CAR_MISSION4_GIMBAL_MID_YAW_SPEED_SPS       (400U) /* 中段强转yaw目标速度。 */
#define CAR_MISSION4_GIMBAL_MID_YAW_ACCEL_STEP_SPS  (80U) /* 中段每1ms最大加速量。 */
#define CAR_MISSION4_GIMBAL_MID_YAW_DECEL_STEP_SPS  (80U) /* 中段每1ms最大减速量。 */

#define CAR_MISSION4_GIMBAL_FAR_YAW_STEPS           (0U) /* 远段每次强转需要额外跟随的yaw步数。 */
#define CAR_MISSION4_GIMBAL_FAR_YAW_SPEED_SPS       (400U) /* 远段强转yaw目标速度。 */
#define CAR_MISSION4_GIMBAL_FAR_YAW_ACCEL_STEP_SPS  (80U) /* 远段每1ms最大加速量。 */
#define CAR_MISSION4_GIMBAL_FAR_YAW_DECEL_STEP_SPS  (80U) /* 远段每1ms最大减速量。 */
#else
/* 固定yaw随动库关闭后的编译占位；这些数值不会进入任务输出。 */
#define CAR_MISSION4_GIMBAL_NEAR_YAW_STEPS           (0U)
#define CAR_MISSION4_GIMBAL_NEAR_YAW_SPEED_SPS         (0)
#define CAR_MISSION4_GIMBAL_NEAR_YAW_ACCEL_STEP_SPS  (80U)
#define CAR_MISSION4_GIMBAL_NEAR_YAW_DECEL_STEP_SPS  (80U)
#define CAR_MISSION4_GIMBAL_MID_YAW_STEPS            (0U)
#define CAR_MISSION4_GIMBAL_MID_YAW_SPEED_SPS          (0)
#define CAR_MISSION4_GIMBAL_MID_YAW_ACCEL_STEP_SPS   (80U)
#define CAR_MISSION4_GIMBAL_MID_YAW_DECEL_STEP_SPS   (80U)
#define CAR_MISSION4_GIMBAL_FAR_YAW_STEPS            (0U)
#define CAR_MISSION4_GIMBAL_FAR_YAW_SPEED_SPS          (0)
#define CAR_MISSION4_GIMBAL_FAR_YAW_ACCEL_STEP_SPS   (80U)
#define CAR_MISSION4_GIMBAL_FAR_YAW_DECEL_STEP_SPS   (80U)
#endif

/* 主循环和 OLED 菜单刷新节拍。 */
#define CAR_APP_LOOP_DELAY_MS          (1U)   /* 裸机兼容循环的基础延时；RTOS任务有独立周期。 */
#define CAR_MENU_REFRESH_MS            (100U) /* 菜单无事件时的常规刷新间隔。 */
/* 把毫秒刷新间隔换算成裸机循环次数；自动派生，不要手动修改。 */
#define CAR_MENU_REFRESH_TICKS \
    ((CAR_MENU_REFRESH_MS + CAR_APP_LOOP_DELAY_MS - 1U) / \
        CAR_APP_LOOP_DELAY_MS)

/*
 * 灰度输入。
 * 数字量模式下 Gray_ReadDigitalMaskFast 直接读 GPIO，NO YAW 定时器中断使用这条快路径。
 */
#define GRAY_DIGITAL_ACTIVE_HIGH       (1U) /* 1：GPIO高电平表示压线；0：低电平表示压线。 */
#define GRAY_DIGITAL_INPUT_PULL_UP     (1U) /* 1：启用MCU内部上拉；模块已带推挽输出时可设0。 */
#define GRAY_SENSOR_COUNT              (7U) /* 灰度通道总数，当前固定S1至S7。 */
#define CAR_GRAY_TRACK_SENSOR_MASK     (0x7FU) /* bit6至bit0对应S1至S7；1表示参与循迹。 */

/* 模拟模式保留编译能力，正式车当前不走 ADC 灰度。 */
#define GRAY_ACTIVE_HIGH               (1U) /* 模拟值高于阈值时是否判为压线；只用于ADC方法。 */
#define GRAY_ADC_MAX_VALUE             (4095U) /* 12位ADC满量程，用于限幅和数字模式兼容值。 */
#define GRAY_DEFAULT_THRESHOLD         (2000U) /* 未校准时各路默认黑白阈值；数字模式不使用。 */
#define GRAY_ADC_TIMEOUT_COUNT         (100000U) /* 等待ADC完成的忙等上限，防止硬件异常死循环。 */
#define GRAY_FILTER_SAMPLE_COUNT       (1U) /* 每次Gray_Update平均的ADC样本数；越大越稳但越慢。 */
#define GRAY_DIGITAL_CONFIRM_COUNT     (1U) /* 单路状态连续出现几次才确认；1表示不额外防抖。 */
#define GRAY_LINE_ERROR_SCALE          (100) /* 加权循迹误差放大倍数，避免使用浮点数。 */

#endif /* CAR_PROFILE_IS_GMR */

#endif
