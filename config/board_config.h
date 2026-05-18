#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

/* CAR_MOTOR_PWM_MAX_COUNTS：电机 PWM 占空比上限，对应 PWM_PERIOD_COUNTS。 */
#define CAR_MOTOR_PWM_MAX_COUNTS        (4000U)

/* CAR_TRACK_BASE_DUTY：基础循迹时的默认电机占空比。 */
#define CAR_TRACK_BASE_DUTY             (900)

/* CAR_TRACK_TURN_GAIN：根据循迹误差计算转向修正的增益。 */
#define CAR_TRACK_TURN_GAIN             (220)

/* CAR_TRACK_LOST_STOP：丢线时是否立刻停车。 */
#define CAR_TRACK_LOST_STOP             (1U)

/* CAR_APP_LOOP_DELAY_MS：主循环延时，避免空转过快。 */
#define CAR_APP_LOOP_DELAY_MS           (10U)

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
