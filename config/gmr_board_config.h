#ifndef GMR_BOARD_CONFIG_H
#define GMR_BOARD_CONFIG_H

/* GMR 保留编码底盘、按键、显示、UART0 调参和 Task4 基础灰度循迹。 */
#define CAR_RECOVERY_SAFE_BUILD        (0U)

/* UART0/PA10、PA11只用于Task2/Task5日志和ASCII调参。 */
#ifndef CAR_ENABLE_LOG_UART
#define CAR_ENABLE_LOG_UART            (1U)
#endif
#ifndef CAR_ENABLE_LOG_UART_RX
#define CAR_ENABLE_LOG_UART_RX         (1U)
#endif

/* 外部M0姿态使用独立UART3：PB2为MCU TX，PB3接收姿态帧。 */
#define GMR_H7_IMU_STALE_MS             (100U)
#define CAR_UART0_TX_REQUIRED           (CAR_ENABLE_LOG_UART != 0U)
#define CAR_UART0_REQUIRED \
    (CAR_UART0_TX_REQUIRED || (CAR_ENABLE_LOG_UART_RX != 0U))

/* 本地 OLED 布局参数。 */
#define CAR_LOCAL_OLED_FONT_SIZE_PIXELS        (12U)
#define CAR_LOCAL_OLED_VISIBLE_ROWS \
    ((uint8_t)(64U / CAR_LOCAL_OLED_FONT_SIZE_PIXELS))
#define CAR_LOCAL_OLED_MAX_CHARS_PER_ROW \
    ((uint8_t)(128U / (CAR_LOCAL_OLED_FONT_SIZE_PIXELS / 2U)))

/* 板载状态灯与按键。 */
#define CAR_ENABLE_DEBUG_LED           (1U)
#define CAR_ENABLE_PA14_DEBUG_LED      CAR_ENABLE_DEBUG_LED
#define CAR_KEY_ACTIVE_LOW             (1U)

/* GMR 暂不启用 UI 看门狗，保留参数供 RTOS 条件编译检查。 */
#define CAR_ENABLE_UI_WATCHDOG         (0U)
#define CAR_WATCHDOG_TASK_PRIORITY     (5U)
#define CAR_WATCHDOG_TASK_STACK_WORDS  (96U)
#define CAR_WATCHDOG_CHECK_PERIOD_MS   (250U)
#define CAR_WATCHDOG_UI_TIMEOUT_MS     (2000U)
#define CAR_WATCHDOG_HW_CLOCK_DIVIDER  DL_WWDT_CLOCK_DIVIDE_8
#define CAR_WATCHDOG_HW_TIMER_PERIOD   DL_WWDT_TIMER_PERIOD_12_BITS

/* 电机和编码器方向必须先用 Task2 单轮确认，再按实际接线调整。 */
#define CAR_CHASSIS_LEFT_REVERSE       (0U)
#define CAR_CHASSIS_RIGHT_REVERSE      (1U)

/* 主循环兼容入口与 Menu2 刷新节拍。 */
#define CAR_APP_LOOP_DELAY_MS          (1U)
#define CAR_MENU_REFRESH_MS            (100U)
#define CAR_MENU_REFRESH_TICKS \
    ((CAR_MENU_REFRESH_MS + CAR_APP_LOOP_DELAY_MS - 1U) / \
        CAR_APP_LOOP_DELAY_MS)

/*
 * Task4 复用普通版 motor_no_yaw 底层模块。
 * 速度单位为 encoder count/20ms，与 Task2 target 命令一致。
 * lineError = 平均权重 * 100；正数降低左轮、提高右轮，车体左转。
 */
#define CAR_MOTOR_NO_YAW_TASK1_PID_BASE_SPEED_COUNTS_PER_PERIOD       (25)
#define CAR_MOTOR_NO_YAW_TASK1_PID_MIN_SPEED_COUNTS_PER_PERIOD        (-10)
#define CAR_MOTOR_NO_YAW_TASK1_PID_MAX_SPEED_COUNTS_PER_PERIOD        (40)
#define CAR_MOTOR_NO_YAW_TASK1_PID_LINE_DEADBAND                      (1)
#define CAR_MOTOR_NO_YAW_TASK1_PID_GRAY_P_GAIN                        (3)
#define CAR_MOTOR_NO_YAW_TASK1_PID_GRAY_D_GAIN                        (0)
#define CAR_MOTOR_NO_YAW_TASK1_PID_CORRECTION_LIMIT_COUNTS            (30U)
#define CAR_MOTOR_NO_YAW_TASK1_PID_SEARCH_SPEED_COUNTS_PER_PERIOD     (10)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S1_WEIGHT                          (13)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S2_WEIGHT                          (11)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S3_WEIGHT                          (3)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S4_WEIGHT                          (0)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S5_WEIGHT                          (-3)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S6_WEIGHT                          (-11)
#define CAR_MOTOR_NO_YAW_TASK1_PID_S7_WEIGHT                          (-13)

#define CAR_MOTOR_NO_YAW_TASK4_PID_BASE_SPEED_COUNTS_PER_PERIOD       (25)
#define CAR_MOTOR_NO_YAW_TASK4_PID_MIN_SPEED_COUNTS_PER_PERIOD        (-10)
#define CAR_MOTOR_NO_YAW_TASK4_PID_MAX_SPEED_COUNTS_PER_PERIOD        (40)
#define CAR_MOTOR_NO_YAW_TASK4_PID_LINE_DEADBAND                      (1)
#define CAR_MOTOR_NO_YAW_TASK4_PID_GRAY_P_GAIN                        (3)
#define CAR_MOTOR_NO_YAW_TASK4_PID_GRAY_D_GAIN                        (0)
#define CAR_MOTOR_NO_YAW_TASK4_PID_CORRECTION_LIMIT_COUNTS            (30U)
#define CAR_MOTOR_NO_YAW_TASK4_PID_SEARCH_SPEED_COUNTS_PER_PERIOD     (10)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S1_WEIGHT                          (13)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S2_WEIGHT                          (11)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S3_WEIGHT                          (3)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S4_WEIGHT                          (0)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S5_WEIGHT                          (-3)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S6_WEIGHT                          (-11)
#define CAR_MOTOR_NO_YAW_TASK4_PID_S7_WEIGHT                          (-13)

/* 普通版直角强转参数：S1/S2 判左直角，S6/S7 判右直角，S1/S7 回线退出。 */
#define CAR_MOTOR_NO_YAW_TASK1_TURN_INNER_SPEED_COUNTS_PER_PERIOD     (-25)
#define CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_SPEED_COUNTS_PER_PERIOD       (25)
#define CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD  (25)
#define CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD      (25)
#define CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD (25)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_MIN_ENCODER_GAP_COUNTS          (3500U)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_APPROACH_MS                       (300U)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD (25)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD (26)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_MS                            (40U)
#define CAR_MOTOR_NO_YAW_TASK1_TURN_HOLD_MS                         (10000U)
#define CAR_MOTOR_NO_YAW_TASK1_LINE_LOST_TIMEOUT_MS                     (0U)

#define CAR_MOTOR_NO_YAW_TASK4_TURN_INNER_SPEED_COUNTS_PER_PERIOD     (-30)
#define CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_SPEED_COUNTS_PER_PERIOD       (30)
#define CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD  (30)
#define CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD      (30)
#define CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD (30)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_MIN_ENCODER_GAP_COUNTS          (3500U)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_APPROACH_MS                       (300U)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD (25)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD (26)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_MS                            (40U)
#define CAR_MOTOR_NO_YAW_TASK4_TURN_HOLD_MS                          (5000U)
#define CAR_MOTOR_NO_YAW_TASK4_LINE_LOST_TIMEOUT_MS                     (0U)

#define CAR_MOTOR_NO_YAW_LEFT_TURN_WINDOW_MS                          (100U)
#define CAR_MOTOR_NO_YAW_RIGHT_TURN_WINDOW_MS                         (100U)
#define CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US                             (1000U)
#define CAR_MOTOR_NO_YAW_LINE_CONFIRM_SAMPLES                           (2U)
#define CAR_MOTOR_NO_YAW_TURN_REARM_MS                                 (20U)
#define CAR_MOTOR_NO_YAW_RETURN_CONFIRM_SAMPLES                         (2U)
#define CAR_MOTOR_NO_YAW_TURN_TARGET_ANGLE_X100                         (1U)

/* 七路数字灰度：bit6..bit0 对应 S1..S7。 */
#define GRAY_DIGITAL_ACTIVE_HIGH       (1U)
#define GRAY_DIGITAL_INPUT_PULL_UP     (1U)
#define GRAY_SENSOR_COUNT              (7U)
#define CAR_GRAY_TRACK_SENSOR_MASK     (0x7FU)
#define GRAY_ACTIVE_HIGH               (1U)
#define GRAY_ADC_MAX_VALUE             (4095U)
#define GRAY_DEFAULT_THRESHOLD         (2000U)
#define GRAY_ADC_TIMEOUT_COUNT         (100000U)
#define GRAY_FILTER_SAMPLE_COUNT       (1U)
#define GRAY_DIGITAL_CONFIRM_COUNT     (1U)
#define GRAY_LINE_ERROR_SCALE          (100)

#if ((CAR_ENABLE_LOG_UART_RX != 0U) && !CAR_UART0_REQUIRED)
#error "GMR PID console requires UART0"
#endif

#endif
