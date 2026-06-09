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
 * CAR_LINK_DEBUG_LOG：把 UART3 Link 收到的完整行转发到 Type-C 日志串口。
 * 1：打印 "[PB3 RX] ..."；0：关闭，避免视觉帧率高时刷屏。
 */
#define CAR_LINK_DEBUG_LOG             (0U)

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

/*
 * 步进速度统一使用 SPS（step per second，每秒 STEP 数）。
 * 以前的 command/4 换算已取消，上层写 5000 就表示 5000 step/s。
 */
#define CAR_STEPPER_SPEED_MIN_SPS       (500U)
#define CAR_STEPPER_SPEED_MAX_SPS       (10000U)
#define CAR_STEPPER_SPEED_STEP_SPS      (500U)

/* 菜单里的底盘无限循迹测试基础速度。 */
#define CAR_MOTOR_TEST_DEFAULT_SPEED_SPS (3000U)

/* 菜单里的底盘无限循迹测试急转速度，内轮反转、外轮正转。 */
#define CAR_MOTOR_TEST_TURN_SPEED_SPS    (1500U)

/* 菜单里的云台 yaw/pitch 测试默认速度。 */
#define CAR_GIMBAL_TEST_YAW_SPEED_SPS    (2500U)
#define CAR_GIMBAL_TEST_PITCH_SPEED_SPS  (1500U)

/* CAR_STEPPER_PULSE_HIGH_TICKS：STEP 高电平保持几个定时器 tick。 */
#define CAR_STEPPER_PULSE_HIGH_TICKS    (1U)

/*
 * CAR_CHASSIS_LEFT/RIGHT_REVERSE：底盘左右电机方向反相。
 * 说明：差速底盘两侧电机通常镜像安装，同样的“前进命令”可能需要相反 DIR 电平。
 * 如果 Track Test 两轮还是方向不对，优先改这里，不要临时改 motion/tracking 逻辑。
 */
#define CAR_CHASSIS_LEFT_REVERSE        (1U)
#define CAR_CHASSIS_RIGHT_REVERSE       (0U)

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

/*
 * CAR_KEY_ACTIVE_LOW：按键是否低电平表示按下。
 * 1：内部上拉，按键按下接地；0：内部下拉，按键按下接 VCC。
 */
#define CAR_KEY_ACTIVE_LOW              (1U)

/*
 * CAR_KEY_DEBUG_LOG：周期打印 PB9/PB8 原始电平。
 * 用来判断是菜单逻辑问题，还是按键输入电平/接线问题。
 */
#define CAR_KEY_DEBUG_LOG               (1U)

/* CAR_KEY_DEBUG_PERIOD_TICKS：按键诊断日志周期，当前 10ms 一轮，50 约 500ms。 */
#define CAR_KEY_DEBUG_PERIOD_TICKS      (50U)

/*
 * CAR_KEY_DEBUG_POLL_LOG：是否周期打印按键电平。
 * 0：只打印 init/change/long，避免日志刷屏拖慢按键采样。
 */
#define CAR_KEY_DEBUG_POLL_LOG          (0U)

/* CAR_GIMBAL_X/Y_CONTROL_SPEED_MAX_SPS：视觉闭环时云台单轴最大速度。 */
#define CAR_GIMBAL_X_CONTROL_SPEED_MAX_SPS (10000U)
#define CAR_GIMBAL_Y_CONTROL_SPEED_MAX_SPS (10000U)

/* CAR_GIMBAL_X/Y_MIN_ACTIVE_SPEED_SPS：云台超过死区后的最小微调速度。 */
#define CAR_GIMBAL_X_MIN_ACTIVE_SPEED_SPS (100U)
#define CAR_GIMBAL_Y_MIN_ACTIVE_SPEED_SPS (30U)

/* CAR_GIMBAL_DEADBAND_X/Y：视觉误差死区，单位为 0.1 像素。 */
#define CAR_GIMBAL_DEADBAND_X           (10U)
#define CAR_GIMBAL_DEADBAND_Y           (10U)

/*
 * CAR_GIMBAL_X/Y_ERROR_OFFSET：安装偏差补偿，单位为 0.1 像素。
 * X 正值让最终点向画面右侧偏，Y 负值让最终点向画面上方偏。
 */
#define CAR_GIMBAL_X_ERROR_OFFSET       (0)
#define CAR_GIMBAL_Y_ERROR_OFFSET       (75)

/* CAR_GIMBAL_GAIN_SCALE：云台比例增益缩放基准。 */
#define CAR_GIMBAL_GAIN_SCALE           (100U)

/* CAR_GIMBAL_X/Y_KP：视觉误差到云台 SPS 的比例增益，误差单位为 0.1 像素。 */
#define CAR_GIMBAL_X_KP                 (200U)
#define CAR_GIMBAL_Y_KP                 (100U)

/* CAR_GIMBAL_X/Y_KD：相邻视觉帧误差变化的阻尼增益。 */
#define CAR_GIMBAL_X_KD                 (0U)
#define CAR_GIMBAL_Y_KD                 (0U)

/* CAR_GIMBAL_X_REVERSE/Y_REVERSE：实车方向相反时改成 1。 */
#define CAR_GIMBAL_X_REVERSE            (1U)
#define CAR_GIMBAL_Y_REVERSE            (0U)

/* CAR_GIMBAL_VISION_TIMEOUT_TICKS：连续多少轮没有视觉更新就停云台。 */
#define CAR_GIMBAL_VISION_TIMEOUT_TICKS (20U)

/* CAR_GIMBAL_TEST_TARGET_X/Y：旧坐标模式默认目标点；当前差值模式不使用。 */
#define CAR_GIMBAL_TEST_TARGET_X        (160)
#define CAR_GIMBAL_TEST_TARGET_Y        (120)

/*
 * CAR_GIMBAL_TEST_USE_CIRCLE_ERROR：视觉脚本发 "中心误差;圆点误差" 时选哪一组。
 * 1：使用圆点误差，适合墙面画圆跟踪。
 * 0：使用矩形中心误差，适合先把激光打到框中心。
 */
#define CAR_GIMBAL_TEST_USE_CIRCLE_ERROR (0U)

/* CAR_GIMBAL_MOTOR_TEST_LR_STEPS_PER_90：云台左右轴转 90 度需要的 STEP 数。 */
#define CAR_GIMBAL_MOTOR_TEST_LR_STEPS_PER_90 (1200U)

/* CAR_GIMBAL_MOTOR_TEST_UD_STEPS_PER_90：云台上下轴转 90 度需要的 STEP 数。 */
#define CAR_GIMBAL_MOTOR_TEST_UD_STEPS_PER_90 (800U)

/* CAR_GIMBAL_PITCH_LIMIT_DEG：视觉闭环时 pitch 相对进入位置的角度限幅。 */
#define CAR_GIMBAL_PITCH_LIMIT_DEG      (50U)

/* CAR_GIMBAL_PITCH_STEPS_PER_90：pitch 轴 90 度对应的 STEP 数，用于限幅估算。 */
#define CAR_GIMBAL_PITCH_STEPS_PER_90   (CAR_GIMBAL_MOTOR_TEST_UD_STEPS_PER_90)

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

/* CAR_GIMBAL_MOTOR_TEST_CIRCLE_SPEED_SPS：开环画圆时两轴的最大速度。 */
#define CAR_GIMBAL_MOTOR_TEST_CIRCLE_SPEED_SPS (2500U)

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

/* CAR_TRACK_BASE_SPEED_SPS：基础循迹时的默认底盘速度。 */
#define CAR_TRACK_BASE_SPEED_SPS        (900U)

/* CAR_TRACK_STEP_TEST_TARGET_STEPS：Track Test 固定输出的底盘平均 STEP 数。 */
#define CAR_TRACK_STEP_TEST_TARGET_STEPS (5000U)

/* CAR_TRACK_TURN_GAIN：根据循迹误差计算转向修正的增益。 */
#define CAR_TRACK_TURN_GAIN             (220)

/* CAR_TRACK_LOST_HOLD_TICKS：刚丢线时先保持上一拍输出的循环次数。 */
#define CAR_TRACK_LOST_HOLD_TICKS       (3U)

/* CAR_TRACK_LOST_SEARCH_TICKS：进入温和搜线动作后的持续次数。 */
#define CAR_TRACK_LOST_SEARCH_TICKS     (24U)

/* CAR_TRACK_LOST_SEARCH_BASE_SPEED_SPS：搜线阶段的基础底盘速度。 */
#define CAR_TRACK_LOST_SEARCH_BASE_SPEED_SPS (650U)

/* CAR_TRACK_LOST_SEARCH_DELTA_SPS：搜线阶段左右轮的差速幅度。 */
#define CAR_TRACK_LOST_SEARCH_DELTA_SPS (180U)

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

/* CAR_ROUTE_CRUISE_SPEED_SPS：直道巡航时的基础速度。 */
#define CAR_ROUTE_CRUISE_SPEED_SPS      (1000U)

/* CAR_ROUTE_APPROACH_SPEED_SPS：接近直角前的降速速度。 */
#define CAR_ROUTE_APPROACH_SPEED_SPS    (820U)

/* CAR_ROUTE_TURN_SPEED_SPS：直角转弯时的低速速度。 */
#define CAR_ROUTE_TURN_SPEED_SPS        (650U)

/* CAR_ROUTE_EXIT_SPEED_SPS：出弯恢复阶段的速度。 */
#define CAR_ROUTE_EXIT_SPEED_SPS        (900U)

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

/*
 * CAR_GRAY_TRACK_SENSOR_MASK：参与循迹的灰度通道 bitmask。
 * bit0~bit6 对应 S1~S7；当前只看中间 S2~S6，忽略 S1/S7。
 */
#define CAR_GRAY_TRACK_SENSOR_MASK      (0x3EU)

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
