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
 * CAR_ENABLE_LOG_UART：是否启用 Type-C CH340 日志串口。
 * UART0 复用 PA10/PA11，和 BSL 串口走同一组物理引脚。
 */
#define CAR_ENABLE_LOG_UART            (1U)

/*
 * CAR_ENABLE_JQ8400：语音模块开关。
 * 当前为了把 UART1 PB6/PB7 分给 JY61P，JQ8400 暂停接入。
 */
#define CAR_ENABLE_JQ8400              (0U)

/*
 * CAR_ENABLE_PA14_DEBUG_LED：是否启用 PA14 状态灯。
 * 1.1ccs 已把灰度 S1 迁到 PA15，PA14 固定作 LED，可用于观察主循环状态。
 */
#define CAR_ENABLE_PA14_DEBUG_LED       (1U)

/* CAR_MOTOR_PWM_MAX_COUNTS：保留旧速度命令量程，1.1ccs 中会换算成步进脉冲频率。 */
#define CAR_MOTOR_PWM_MAX_COUNTS        (4000U)

/* CAR_STEPPER_MAX_COMMAND：步进电机速度命令最大值，沿用 0~4000 的旧量程。 */
#define CAR_STEPPER_MAX_COMMAND         (4000U)

/* CAR_STEPPER_MAX_STEPS_PER_TASK：每次 Motor_Task 单个电机最多补发多少个 STEP。 */
#define CAR_STEPPER_MAX_STEPS_PER_TASK  (4U)

/* CAR_STEPPER_PULSE_CYCLES：STEP 高电平保持时间，32MHz 下约 10us。 */
#define CAR_STEPPER_PULSE_CYCLES        (320U)

/* CAR_STEPPER_TEST_COMMAND：方向测试时的低速步进命令。 */
#define CAR_STEPPER_TEST_COMMAND        (1200U)

/* CAR_TRACK_BASE_DUTY：基础循迹时的默认电机占空比。 */
#define CAR_TRACK_BASE_DUTY             (900)

/* CAR_TRACK_TURN_GAIN：根据循迹误差计算转向修正的增益。 */
#define CAR_TRACK_TURN_GAIN             (220)

/* CAR_TRACK_LOST_HOLD_TICKS：刚丢线时先保持上一拍输出的循环次数。 */
#define CAR_TRACK_LOST_HOLD_TICKS       (3U)

/* CAR_TRACK_LOST_SEARCH_TICKS：进入温和搜线动作后的持续次数。 */
#define CAR_TRACK_LOST_SEARCH_TICKS     (24U)

/* CAR_TRACK_LOST_SEARCH_BASE_DUTY：搜线阶段的基础占空比。 */
#define CAR_TRACK_LOST_SEARCH_BASE_DUTY (650U)

/* CAR_TRACK_LOST_SEARCH_DELTA_DUTY：搜线阶段左右轮的差速幅度。 */
#define CAR_TRACK_LOST_SEARCH_DELTA_DUTY (180U)

/* CAR_TRACK_ADC_FAULT_STOP_TICKS：ADC 连续失败多少次后判定为异常停车。 */
#define CAR_TRACK_ADC_FAULT_STOP_TICKS  (2U)

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

/*
 * CAR_ENABLE_SPEED_CONTROL：当前四步进方案不再使用外部编码器闭环。
 * 闭环由步进驱动器内部完成，MCU 只输出 STEP/DIR。
 */
#define CAR_ENABLE_SPEED_CONTROL        (0U)

/* CAR_ENABLE_ENCODER_INPUTS：PA12/PA13/PA22 已改作步进控制，编码器输入默认关闭。 */
#define CAR_ENABLE_ENCODER_INPUTS       (0U)

/* CAR_SPEED_CONTROL_PERIOD_MS：速度闭环控制周期，先用 20ms，实车再调。 */
#define CAR_SPEED_CONTROL_PERIOD_MS     (20U)

/* CAR_SPEED_CONTROL_PERIOD_TICKS：把速度闭环周期换算成 App_Task 调度次数。 */
#define CAR_SPEED_CONTROL_PERIOD_TICKS \
    ((CAR_SPEED_CONTROL_PERIOD_MS + CAR_APP_LOOP_DELAY_MS - 1U) / \
        CAR_APP_LOOP_DELAY_MS)

/* CAR_SPEED_MAX_TARGET_TICKS：满 PWM 对应的单周期目标编码器计数，后续按实车调。 */
#define CAR_SPEED_MAX_TARGET_TICKS      (40)

/* CAR_SPEED_TARGET_STEP：每个速度控制周期目标命令最多变化多少，避免突然加速。 */
#define CAR_SPEED_TARGET_STEP           (120)

/* CAR_SPEED_KP：速度 PI 的比例系数，误差单位是单周期编码器计数。 */
#define CAR_SPEED_KP                    (24)

/* CAR_SPEED_KI：速度 PI 的积分系数，用于补偿左右电机差异和低速死区。 */
#define CAR_SPEED_KI                    (2)

/* CAR_SPEED_INTEGRAL_LIMIT：速度 PI 积分限幅，防止长时间堵转后输出冲太大。 */
#define CAR_SPEED_INTEGRAL_LIMIT        (300)

/* CAR_SPEED_MIN_ACTIVE_DUTY：目标非零时的最小有效 PWM，低于它电机可能不动。 */
#define CAR_SPEED_MIN_ACTIVE_DUTY       (450U)

/* CAR_SPEED_LEFT_ENCODER_SIGN：左编码器方向修正，实车反了就改成 -1。 */
#define CAR_SPEED_LEFT_ENCODER_SIGN     (1)

/* CAR_SPEED_RIGHT_ENCODER_SIGN：右编码器方向修正，实车反了就改成 -1。 */
#define CAR_SPEED_RIGHT_ENCODER_SIGN    (1)

/* CAR_ROUTE_EDGE_TICKS：单边直线路程的编码器相对计数，后续按实车调。 */
#define CAR_ROUTE_EDGE_TICKS            (2400U)

/* CAR_ROUTE_APPROACH_TICKS：开始提前降速的路程阈值。 */
#define CAR_ROUTE_APPROACH_TICKS        (1800U)

/* CAR_ROUTE_TURN_HOLD_TICKS：没有 yaw 数据时，拐角状态保留的循环次数。 */
#define CAR_ROUTE_TURN_HOLD_TICKS       (40U)

/* CAR_ROUTE_EXIT_TICKS：出弯后继续恢复速度的循环次数。 */
#define CAR_ROUTE_EXIT_TICKS            (20U)

/* CAR_ROUTE_CRUISE_DUTY：直道巡航时的基础占空比。 */
#define CAR_ROUTE_CRUISE_DUTY           (1000U)

/* CAR_ROUTE_APPROACH_DUTY：接近直角前的降速占空比。 */
#define CAR_ROUTE_APPROACH_DUTY         (820U)

/* CAR_ROUTE_TURN_DUTY：直角转弯时的低速占空比。 */
#define CAR_ROUTE_TURN_DUTY             (650U)

/* CAR_ROUTE_EXIT_DUTY：出弯恢复阶段的占空比。 */
#define CAR_ROUTE_EXIT_DUTY             (900U)

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

/* CAR_ENABLE_MOTOR_TEST_MODE：1 表示按键1进入电机方向确认，平时保持 0。 */
#define CAR_ENABLE_MOTOR_TEST_MODE      (0U)

/* CAR_MOTOR_TEST_DUTY：旧 PWM 测试量程保留项，1.1ccs 步进测试使用 CAR_STEPPER_TEST_COMMAND。 */
#define CAR_MOTOR_TEST_DUTY             (700U)

/* CAR_MOTOR_TEST_RUN_MS：每个方向测试步骤持续时间，到时自动停车。 */
#define CAR_MOTOR_TEST_RUN_MS           (800U)

/* CAR_MOTOR_TEST_RUN_TICKS：把测试时间换算成 App_Task 调度次数。 */
#define CAR_MOTOR_TEST_RUN_TICKS \
    ((CAR_MOTOR_TEST_RUN_MS + CAR_APP_LOOP_DELAY_MS - 1U) / CAR_APP_LOOP_DELAY_MS)

/* GRAY_SENSOR_COUNT：灰度传感器通道总数。 */
#define GRAY_SENSOR_COUNT               (7U)

/* GRAY_ACTIVE_HIGH：1 表示 ADC 越大越像压线，0 表示相反。 */
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
