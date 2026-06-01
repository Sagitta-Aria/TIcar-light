#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

/*
 * CAR_RECOVERY_SAFE_BUILD：芯片恢复用安全构建开关。
 *
 * 1：只初始化 PA14 状态灯和三路 UART 心跳，不进入 App，不初始化 OLED/I2C/PLL/ADC/电机。
 *    用于刚解锁芯片后的第一次下载，确认芯片和调试链路恢复稳定。
 * 0：恢复正常小车固件。
 *
 * 注意：只有救板子或首次恢复下载时才改成 1，正常小车固件保持 0。
 */
#define CAR_RECOVERY_SAFE_BUILD       (0U)

/*
 * CAR_GIMBAL_PIN_TEST_BUILD：云台 STEP/DIR 最小引脚测试固件。
 *
 * 1：不初始化 OLED/I2C、UART、灰度、视觉、菜单、TIMG0 步进调度。
 *    只使用 GPIO 直接输出云台两轴 STEP/DIR 脉冲，用来排除复杂工程逻辑。
 * 0：正常小车固件。
 *
 * 当前用于排查云台驱动器接线和芯片异常，测试完成后应改回 0。
 */
#define CAR_GIMBAL_PIN_TEST_BUILD     (0U)

#if (CAR_RECOVERY_SAFE_BUILD != 0U) && (CAR_GIMBAL_PIN_TEST_BUILD != 0U)
#error "CAR_RECOVERY_SAFE_BUILD and CAR_GIMBAL_PIN_TEST_BUILD cannot both be enabled"
#endif

/*
 * CAR_ENABLE_LOG_UART：工程日志总开关。
 *
 * 1：启用 Type-C CH340 日志串口，启动、按键、状态机、路线阶段、循迹异常等行为都会打印。
 * 0：LOG_* 宏编译为空操作，正常小车逻辑不再输出日志。
 *
 * UART0 复用 PA10/PA11，和 BSL 串口走同一组物理引脚。
 * 注意：PA10/PA11 仍按接线表保留给 Type-C/BSL，不建议拿去接其它外设。
 */
#define CAR_ENABLE_LOG_UART            (1U)

/*
 * CAR_ENABLE_JQ8400：语音模块开关。
 * 当前为了把 UART1 PB6/PB7 分给 JY61P，JQ8400 暂停接入。
 */
#define CAR_ENABLE_JQ8400              (0U)

/*
 * CAR_ENABLE_PA14_DEBUG_LED：是否启用 PA14 状态灯。
 * ccs1.2 已把灰度 S1 迁到 PA15，PA14 固定作 LED，可用于观察主循环状态。
 */
#define CAR_ENABLE_PA14_DEBUG_LED       (1U)

/* CAR_MOTOR_COMMAND_MAX：步进速度命令最大值，0~4000 会换算成 STEP 频率。 */
#define CAR_MOTOR_COMMAND_MAX           (4000U)

/*
 * CAR_STEPPER_COMMAND_TO_HZ_DIVISOR：速度命令到 STEP 频率的换算比例。
 * 当前 4000 命令约等于 400 step/s，保持上一版低速测试手感。
 */
#define CAR_STEPPER_COMMAND_TO_HZ_DIVISOR (10U)

/* CAR_STEPPER_PULSE_HIGH_TICKS：STEP 高电平保持几个定时器 tick。 */
#define CAR_STEPPER_PULSE_HIGH_TICKS    (1U)

/*
 * CAR_STEPPER_ENABLE_ACTIVE_LOW：步进驱动器 EN 使能电平。
 * ZDT 官方 PUL 示例默认 En_Pin 拉低使能，因此这里先按低有效测试。
 */
#define CAR_STEPPER_ENABLE_ACTIVE_LOW   (1U)

/*
 * CAR_STEPPER_ENABLE_DEFAULT_ON：上电后是否默认使能四个驱动器。
 * 当前用于查线，默认保持 0；进入云台 Enable Test 菜单后才拉使能。
 */
#define CAR_STEPPER_ENABLE_DEFAULT_ON   (0U)

/* CAR_GIMBAL_COMMAND_MAX：云台单轴最大速度命令。 */
#define CAR_GIMBAL_COMMAND_MAX          (1800U)

/* CAR_GIMBAL_MIN_ACTIVE_COMMAND：云台超过死区后的最小启动命令。 */
#define CAR_GIMBAL_MIN_ACTIVE_COMMAND   (120U)

/* CAR_GIMBAL_DEADBAND_X/Y：视觉误差死区，单位由视觉坐标决定。 */
#define CAR_GIMBAL_DEADBAND_X           (3U)
#define CAR_GIMBAL_DEADBAND_Y           (3U)

/* CAR_GIMBAL_GAIN_SCALE：云台比例增益缩放基准。 */
#define CAR_GIMBAL_GAIN_SCALE           (100U)

/* CAR_GIMBAL_X_KP/Y_KP：视觉误差到 STEP 命令的比例增益。 */
#define CAR_GIMBAL_X_KP                 (80U)
#define CAR_GIMBAL_Y_KP                 (80U)

/* CAR_GIMBAL_X_REVERSE/Y_REVERSE：实车方向相反时改成 1。 */
#define CAR_GIMBAL_X_REVERSE            (0U)
#define CAR_GIMBAL_Y_REVERSE            (0U)

/* CAR_GIMBAL_VISION_TIMEOUT_TICKS：连续多少轮没有视觉更新就停云台。 */
#define CAR_GIMBAL_VISION_TIMEOUT_TICKS (20U)

/* CAR_GIMBAL_TEST_TARGET_X/Y：云台测试中只收到当前位置时使用的默认目标点。 */
#define CAR_GIMBAL_TEST_TARGET_X        (160)
#define CAR_GIMBAL_TEST_TARGET_Y        (120)

/* CAR_GIMBAL_MOTOR_TEST_COMMAND：云台电机测试的慢速命令。 */
#define CAR_GIMBAL_MOTOR_TEST_COMMAND   (2000U)

/* CAR_GIMBAL_MOTOR_TEST_LR_STEPS_PER_90：云台左右轴转 90 度需要的 STEP 数。 */
#define CAR_GIMBAL_MOTOR_TEST_LR_STEPS_PER_90 (1200U)

/* CAR_GIMBAL_MOTOR_TEST_UD_STEPS_PER_90：云台上下轴转 90 度需要的 STEP 数。 */
#define CAR_GIMBAL_MOTOR_TEST_UD_STEPS_PER_90 (800U)

/* CAR_GIMBAL_MOTOR_TEST_ZERO_TICKS：上电记零状态保持多少轮主循环。 */
#define CAR_GIMBAL_MOTOR_TEST_ZERO_TICKS (20U)

/* CAR_GIMBAL_MOTOR_TEST_REVERSE：云台电机测试方向反了就改成 1。 */
#define CAR_GIMBAL_MOTOR_TEST_REVERSE   (0U)

/* CAR_GIMBAL_MOTOR_TEST_RUN_UP_DOWN：1 表示左右轴完成后继续测试上下轴。 */
#define CAR_GIMBAL_MOTOR_TEST_RUN_UP_DOWN (1U)

/* CAR_GIMBAL_MOTOR_TEST_RUN_SIM_TRACK：1 表示 90 度测试后继续跑仿真视觉跟踪。 */
#define CAR_GIMBAL_MOTOR_TEST_RUN_SIM_TRACK (1U)

/* CAR_GIMBAL_MOTOR_TEST_RUN_CIRCLE：1 表示仿真视觉跟踪后继续跑开环画圆。 */
#define CAR_GIMBAL_MOTOR_TEST_RUN_CIRCLE (1U)

/* CAR_GIMBAL_MOTOR_TEST_SIM_HOLD_TICKS：每个仿真视觉点保持多少轮主循环。 */
#define CAR_GIMBAL_MOTOR_TEST_SIM_HOLD_TICKS (40U)

/* CAR_GIMBAL_MOTOR_TEST_CIRCLE_COMMAND：开环画圆时两轴的最大速度命令。 */
#define CAR_GIMBAL_MOTOR_TEST_CIRCLE_COMMAND (900U)

/* CAR_GIMBAL_MOTOR_TEST_CIRCLE_PHASE_TICKS：画圆相位表每一格保持多少轮主循环。 */
#define CAR_GIMBAL_MOTOR_TEST_CIRCLE_PHASE_TICKS (12U)

/* CAR_GIMBAL_MOTOR_TEST_CIRCLE_CYCLES：画圆测试跑几圈。 */
#define CAR_GIMBAL_MOTOR_TEST_CIRCLE_CYCLES (2U)

/* 云台最小引脚测试：每段动作的 STEP 数、脉冲间隔和换向停顿。 */
#define CAR_GIMBAL_PIN_TEST_STEPS_PER_MOVE (400U)
#define CAR_GIMBAL_PIN_TEST_PERIOD_MS      (4U)
#define CAR_GIMBAL_PIN_TEST_PAUSE_TICKS    (80U)

/*
 * CAR_POSE_STEP_TO_MM_NUMERATOR/DENOMINATOR：底盘 STEP 到毫米的换算比例。
 * 当前先按 4 step = 1 mm 占位，实车用尺子标定后再改。
 */
#define CAR_POSE_STEP_TO_MM_NUMERATOR   (1)
#define CAR_POSE_STEP_TO_MM_DENOMINATOR (4)

/* CAR_TRACK_BASE_COMMAND：基础循迹时的默认底盘速度命令。 */
#define CAR_TRACK_BASE_COMMAND          (900)

/* CAR_TRACK_TURN_GAIN：根据循迹误差计算转向修正的增益。 */
#define CAR_TRACK_TURN_GAIN             (220)

/* CAR_TRACK_LOST_HOLD_TICKS：刚丢线时先保持上一拍输出的循环次数。 */
#define CAR_TRACK_LOST_HOLD_TICKS       (3U)

/* CAR_TRACK_LOST_SEARCH_TICKS：进入温和搜线动作后的持续次数。 */
#define CAR_TRACK_LOST_SEARCH_TICKS     (24U)

/* CAR_TRACK_LOST_SEARCH_BASE_COMMAND：搜线阶段的基础底盘速度命令。 */
#define CAR_TRACK_LOST_SEARCH_BASE_COMMAND (650U)

/* CAR_TRACK_LOST_SEARCH_DELTA_COMMAND：搜线阶段左右轮的差速幅度。 */
#define CAR_TRACK_LOST_SEARCH_DELTA_COMMAND (180U)

/* CAR_TRACK_SENSOR_FAULT_STOP_TICKS：灰度采样连续失败多少次后判定为异常停车。 */
#define CAR_TRACK_SENSOR_FAULT_STOP_TICKS (2U)

/* CAR_TRACK_WIDE_LINE_ACTIVE_COUNT：认为进入路口/宽线的最小有效通道数。 */
#define CAR_TRACK_WIDE_LINE_ACTIVE_COUNT (5U)

/* CAR_TRACK_SEARCH_DEFAULT_LEFT：丢线后没有历史方向时默认向左搜线。 */
#define CAR_TRACK_SEARCH_DEFAULT_LEFT   (1U)

/* CAR_TRACK_LOST_STOP：丢线搜线超时后是否停车，1 停车，0 继续搜线。 */
#define CAR_TRACK_LOST_STOP             (1U)

/* CAR_APP_LOOP_DELAY_MS：主循环延时，避免空转过快。 */
#define CAR_APP_LOOP_DELAY_MS           (10U)

/* CAR_MENU_REFRESH_MS：菜单和监视页面的 OLED 刷新周期。 */
#define CAR_MENU_REFRESH_MS             (100U)

/* CAR_MENU_REFRESH_TICKS：把菜单刷新周期换算成 App_Task 调度次数。 */
#define CAR_MENU_REFRESH_TICKS \
    ((CAR_MENU_REFRESH_MS + CAR_APP_LOOP_DELAY_MS - 1U) / \
        CAR_APP_LOOP_DELAY_MS)

/* CAR_MENU_LINK_PRINT_MS：监视页面通过串口打印数据的周期。 */
#define CAR_MENU_LINK_PRINT_MS          (200U)

/* CAR_MENU_LINK_PRINT_TICKS：把串口打印周期换算成 App_Task 调度次数。 */
#define CAR_MENU_LINK_PRINT_TICKS \
    ((CAR_MENU_LINK_PRINT_MS + CAR_APP_LOOP_DELAY_MS - 1U) / \
        CAR_APP_LOOP_DELAY_MS)

/* CAR_ROUTE_EDGE_STEPS：单边直线路程的 STEP 相对计数，后续按实车标定。 */
#define CAR_ROUTE_EDGE_STEPS            (2400U)

/* CAR_ROUTE_APPROACH_STEPS：开始提前降速的 STEP 路程阈值。 */
#define CAR_ROUTE_APPROACH_STEPS        (1800U)

/* CAR_ROUTE_TURN_HOLD_TICKS：没有 yaw 数据时，拐角状态保留的循环次数。 */
#define CAR_ROUTE_TURN_HOLD_TICKS       (40U)

/* CAR_ROUTE_EXIT_TICKS：出弯后继续恢复速度的循环次数。 */
#define CAR_ROUTE_EXIT_TICKS            (20U)

/* CAR_ROUTE_CRUISE_COMMAND：直道巡航时的基础速度命令。 */
#define CAR_ROUTE_CRUISE_COMMAND        (1000U)

/* CAR_ROUTE_APPROACH_COMMAND：接近直角前的降速命令。 */
#define CAR_ROUTE_APPROACH_COMMAND      (820U)

/* CAR_ROUTE_TURN_COMMAND：直角转弯时的低速命令。 */
#define CAR_ROUTE_TURN_COMMAND          (650U)

/* CAR_ROUTE_EXIT_COMMAND：出弯恢复阶段的速度命令。 */
#define CAR_ROUTE_EXIT_COMMAND          (900U)

/* CAR_ROUTE_CRUISE_TURN_LIMIT：直道时允许的最大转向修正。 */
#define CAR_ROUTE_CRUISE_TURN_LIMIT     (1200U)

/* CAR_ROUTE_APPROACH_TURN_LIMIT：接近拐角时允许的最大转向修正。 */
#define CAR_ROUTE_APPROACH_TURN_LIMIT   (900U)

/* CAR_ROUTE_TURN_LIMIT：拐角转弯时允许的最大转向修正。 */
#define CAR_ROUTE_TURN_LIMIT            (700U)

/* CAR_ROUTE_TURN_TOLERANCE_DEG：判断拐角是否转够 90 度的角度误差。 */
#define CAR_ROUTE_TURN_TOLERANCE_DEG    (8)

/* CAR_ROUTE_TURN_IS_LEFT：1 表示默认左转，0 表示默认右转。 */
#define CAR_ROUTE_TURN_IS_LEFT          (1U)

/* CAR_ROUTE_PROFILE_STEP：路线层每次向目标速度平滑靠近的步进。 */
#define CAR_ROUTE_PROFILE_STEP          (20U)

/* CAR_ROUTE_CORNER_COUNT：正方形赛道的直角数量。 */
#define CAR_ROUTE_CORNER_COUNT          (4U)

/*
 * CAR_GRAY_INPUT_DIGITAL：灰度模块输入模式。
 * 1：模块已经把黑白比较做好，MCU 直接读取数字 GPIO 高低电平。
 * 0：模块输出模拟电压，MCU 通过 ADC 原始值和阈值转换黑白。
 */
#define CAR_GRAY_INPUT_DIGITAL         (1U)

/*
 * GRAY_DIGITAL_ACTIVE_HIGH：数字灰度输入的有效电平。
 * 1：GPIO 高电平表示压到黑线。
 * 0：GPIO 低电平表示压到黑线。当前按“灯灭为黑、灯亮为白”的常见接法先用 0。
 */
#define GRAY_DIGITAL_ACTIVE_HIGH       (0U)

/*
 * GRAY_DIGITAL_INPUT_PULL_UP：数字灰度输入是否打开内部弱上拉。
 * 对开漏/比较器输出更稳；若模块是强推挽输出，弱上拉通常也不影响。
 */
#define GRAY_DIGITAL_INPUT_PULL_UP     (1U)

/* GRAY_SENSOR_COUNT：灰度传感器通道总数。 */
#define GRAY_SENSOR_COUNT               (7U)

/* GRAY_ACTIVE_HIGH：模拟 ADC 模式下，1 表示 ADC 越大越像压线，0 表示相反。 */
#define GRAY_ACTIVE_HIGH                (1U)

/* GRAY_ADC_MAX_VALUE：12 位 ADC 的最大值。 */
#define GRAY_ADC_MAX_VALUE              (4095U)

/* GRAY_DEFAULT_THRESHOLD：未校准前的默认阈值。 */
#define GRAY_DEFAULT_THRESHOLD          (2000U)

/* GRAY_ADC_TIMEOUT_COUNT：等待 ADC 转换完成的超时保护计数。 */
#define GRAY_ADC_TIMEOUT_COUNT          (100000U)

/* GRAY_FILTER_SAMPLE_COUNT：每次 Gray_Update 做几次采样平均。 */
#define GRAY_FILTER_SAMPLE_COUNT        (5U)

/* GRAY_DIGITAL_CONFIRM_COUNT：黑白状态变化前需要连续确认几次。 */
#define GRAY_DIGITAL_CONFIRM_COUNT      (2U)

/* GRAY_LINE_ERROR_SCALE：加权误差缩放因子，便于后续调参。 */
#define GRAY_LINE_ERROR_SCALE           (100)

#endif
