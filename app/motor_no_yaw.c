#include "motor_no_yaw.h"

#include "board_config.h"
#include "control_config.h"
#include "gray.h"
#include "jy61p.h"
#include "motion.h"
#include "motor.h"
#include "motor_enable.h"
#include "ti_msp_dl_config.h"

#define MOTOR_NO_YAW_S1_MASK        (0x40U)
#define MOTOR_NO_YAW_S2_MASK        (0x20U)
#define MOTOR_NO_YAW_S3_MASK        (0x10U)
#define MOTOR_NO_YAW_S4_MASK        (0x08U)
#define MOTOR_NO_YAW_S5_MASK        (0x04U)
#define MOTOR_NO_YAW_S6_MASK        (0x02U)
#define MOTOR_NO_YAW_S7_MASK        (0x01U)

#define MOTOR_NO_YAW_LEFT_TURN_NEAR_MASK   MOTOR_NO_YAW_S2_MASK
#define MOTOR_NO_YAW_LEFT_TURN_OUTER_MASK  MOTOR_NO_YAW_S1_MASK
#define MOTOR_NO_YAW_RIGHT_TURN_NEAR_MASK  MOTOR_NO_YAW_S6_MASK
#define MOTOR_NO_YAW_RIGHT_TURN_OUTER_MASK MOTOR_NO_YAW_S7_MASK
#define MOTOR_NO_YAW_LEFT_RETURN_MASK  MOTOR_NO_YAW_LEFT_TURN_OUTER_MASK
#define MOTOR_NO_YAW_RIGHT_RETURN_MASK MOTOR_NO_YAW_RIGHT_TURN_OUTER_MASK
#define MOTOR_NO_YAW_ALL_TURN_MASK \
    (MOTOR_NO_YAW_LEFT_TURN_NEAR_MASK | \
        MOTOR_NO_YAW_LEFT_TURN_OUTER_MASK | \
        MOTOR_NO_YAW_RIGHT_TURN_NEAR_MASK | \
        MOTOR_NO_YAW_RIGHT_TURN_OUTER_MASK)

#define MOTOR_NO_YAW_INVALID_TICKS  (0xFFFFU)
#define MOTOR_NO_YAW_ABS_TARGET(value) \
    (((value) < 0) ? -(value) : (value))

#if (CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US == 0U)
#error "CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US must be greater than 0"
#endif

#if ((CAR_MOTOR_NO_YAW_LINE_CONFIRM_SAMPLES == 0U) || \
    (CAR_MOTOR_NO_YAW_LINE_CONFIRM_SAMPLES > 255U))
#error "CAR_MOTOR_NO_YAW_LINE_CONFIRM_SAMPLES must be 1..255"
#endif

#if ((CAR_MOTOR_NO_YAW_TURN_TARGET_ANGLE_X100 == 0U) || \
    (CAR_MOTOR_NO_YAW_TURN_TARGET_ANGLE_X100 > 18000U))
#error "JY61 turn target must be 0.01..180.00 degrees"
#endif

#if ((CAR_MOTOR_NO_YAW_RETURN_CONFIRM_SAMPLES == 0U) || \
    (CAR_MOTOR_NO_YAW_RETURN_CONFIRM_SAMPLES > 255U))
#error "CAR_MOTOR_NO_YAW_RETURN_CONFIRM_SAMPLES must be 1..255"
#endif

#if CAR_MOTOR_NO_YAW_USE_SPEED_PID

#if ((CAR_MOTOR_NO_YAW_TASK1_PID_MIN_SPEED_COUNTS_PER_PERIOD > \
        CAR_MOTOR_NO_YAW_TASK1_PID_BASE_SPEED_COUNTS_PER_PERIOD) || \
    (CAR_MOTOR_NO_YAW_TASK1_PID_BASE_SPEED_COUNTS_PER_PERIOD > \
        CAR_MOTOR_NO_YAW_TASK1_PID_MAX_SPEED_COUNTS_PER_PERIOD))
#error "Task1 PID line speeds must satisfy min <= base <= max"
#endif

#if ((MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK1_PID_MIN_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK1_PID_MAX_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (CAR_MOTOR_NO_YAW_TASK1_PID_CORRECTION_LIMIT_COUNTS > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK1_PID_SEARCH_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD))
#error "Task1 PID tracking parameters exceed the target limit"
#endif

#if ((CAR_MOTOR_NO_YAW_TASK4_PID_MIN_SPEED_COUNTS_PER_PERIOD > \
        CAR_MOTOR_NO_YAW_TASK4_PID_BASE_SPEED_COUNTS_PER_PERIOD) || \
    (CAR_MOTOR_NO_YAW_TASK4_PID_BASE_SPEED_COUNTS_PER_PERIOD > \
        CAR_MOTOR_NO_YAW_TASK4_PID_MAX_SPEED_COUNTS_PER_PERIOD))
#error "Task4 PID line speeds must satisfy min <= base <= max"
#endif

#if ((MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK4_PID_MIN_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK4_PID_MAX_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (CAR_MOTOR_NO_YAW_TASK4_PID_CORRECTION_LIMIT_COUNTS > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK4_PID_SEARCH_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD))
#error "Task4 PID tracking parameters exceed the target limit"
#endif

#else

#if ((CAR_MOTOR_NO_YAW_TASK1_PWM_BASE_COUNTS < 0) || \
    (CAR_MOTOR_NO_YAW_TASK1_PWM_BASE_COUNTS > CHASSIS_PWM_LIMIT_COUNTS) || \
    (CAR_MOTOR_NO_YAW_TASK1_PWM_SEARCH_COUNTS < 0) || \
    (CAR_MOTOR_NO_YAW_TASK1_PWM_SEARCH_COUNTS > CHASSIS_PWM_LIMIT_COUNTS) || \
    (CAR_MOTOR_NO_YAW_TASK1_PWM_GRAY_GAIN < 0) || \
    (CAR_MOTOR_NO_YAW_TASK1_PWM_GRAY_LIMIT_COUNTS > \
        CHASSIS_PWM_LIMIT_COUNTS) || \
    (CAR_MOTOR_NO_YAW_TASK1_PWM_SYNC_GAIN_Q1024 < 0L) || \
    (CAR_MOTOR_NO_YAW_TASK1_PWM_SYNC_LIMIT_COUNTS > \
        CHASSIS_PWM_LIMIT_COUNTS))
#error "Task1 direct PWM tracking parameters are invalid"
#endif

#if ((CAR_MOTOR_NO_YAW_TASK4_PWM_BASE_COUNTS < 0) || \
    (CAR_MOTOR_NO_YAW_TASK4_PWM_BASE_COUNTS > CHASSIS_PWM_LIMIT_COUNTS) || \
    (CAR_MOTOR_NO_YAW_TASK4_PWM_SEARCH_COUNTS < 0) || \
    (CAR_MOTOR_NO_YAW_TASK4_PWM_SEARCH_COUNTS > CHASSIS_PWM_LIMIT_COUNTS) || \
    (CAR_MOTOR_NO_YAW_TASK4_PWM_GRAY_GAIN < 0) || \
    (CAR_MOTOR_NO_YAW_TASK4_PWM_GRAY_LIMIT_COUNTS > \
        CHASSIS_PWM_LIMIT_COUNTS) || \
    (CAR_MOTOR_NO_YAW_TASK4_PWM_SYNC_GAIN_Q1024 < 0L) || \
    (CAR_MOTOR_NO_YAW_TASK4_PWM_SYNC_LIMIT_COUNTS > \
        CHASSIS_PWM_LIMIT_COUNTS))
#error "Task4 direct PWM tracking parameters are invalid"
#endif

#endif

#if ((MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK1_TURN_INNER_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD))
#error "Task1 turn targets exceed the chassis target limit"
#endif

#if ((MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK4_TURN_INNER_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD))
#error "Task4 turn targets exceed the chassis target limit"
#endif

#define MOTOR_NO_YAW_TIMER_DIV_TICKS \
    (((GRAY_SAMPLE_TIMER_TICK_HZ * CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US) + \
        999999U) / 1000000U)

#if (MOTOR_NO_YAW_TIMER_DIV_TICKS == 0U)
#error "MOTOR_NO_YAW_TIMER_DIV_TICKS must be greater than 0"
#endif

#define MOTOR_NO_YAW_LEFT_TURN_WINDOW_SAMPLES \
    (((CAR_MOTOR_NO_YAW_LEFT_TURN_WINDOW_MS * 1000U) + \
        CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US - 1U) / \
        CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US)

#define MOTOR_NO_YAW_RIGHT_TURN_WINDOW_SAMPLES \
    (((CAR_MOTOR_NO_YAW_RIGHT_TURN_WINDOW_MS * 1000U) + \
        CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US - 1U) / \
        CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US)

#define MOTOR_NO_YAW_TURN_REARM_SAMPLES \
    (((CAR_MOTOR_NO_YAW_TURN_REARM_MS * 1000U) + \
        CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US - 1U) / \
        CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US)

#if ((MOTOR_NO_YAW_LEFT_TURN_WINDOW_SAMPLES == 0U) || \
    (MOTOR_NO_YAW_LEFT_TURN_WINDOW_SAMPLES > 0xFFFFU) || \
    (MOTOR_NO_YAW_RIGHT_TURN_WINDOW_SAMPLES == 0U) || \
    (MOTOR_NO_YAW_RIGHT_TURN_WINDOW_SAMPLES > 0xFFFFU))
#error "Turn window samples must fit uint16_t and be greater than zero"
#endif

#if ((MOTOR_NO_YAW_TURN_REARM_SAMPLES == 0U) || \
    (MOTOR_NO_YAW_TURN_REARM_SAMPLES > 0xFFFFU))
#error "Task1 turn rearm samples must fit uint16_t and be greater than zero"
#endif

typedef enum {
    MOTOR_NO_YAW_PROFILE_TASK1 = 0,
    MOTOR_NO_YAW_PROFILE_TASK4,
    MOTOR_NO_YAW_PROFILE_COUNT
} MotorNoYawProfile;

typedef enum {
    MOTOR_NO_YAW_TURN_NONE = 0,
    MOTOR_NO_YAW_TURN_LEFT,
    MOTOR_NO_YAW_TURN_RIGHT
} MotorNoYawTurnDirection;

/* 编译时只保留所选算法字段；数组中的两个元素分别属于Task1和Task4。 */
typedef struct {
    int16_t lineWeight[GRAY_SENSOR_COUNT];
    int16_t lineDeadband;
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
    int16_t baseSpeedCounts;
    int16_t minLineSpeedCounts;
    int16_t maxLineSpeedCounts;
    int16_t turnGain;
    int16_t turnDGain;
    uint16_t turnLimitCounts;
    int16_t defaultSearchSpeedCounts;
#else
    int16_t basePwmCounts;
    int16_t searchPwmCounts;
    int16_t grayGain;
    uint16_t grayLimitPwmCounts;
    int32_t syncGainQ1024;
    uint16_t syncLimitPwmCounts;
#endif
    int16_t turnInnerSpeedCounts;
    int16_t leftTurnSpeedCounts;
    int16_t leftTurnNearSpeedCounts;
    int16_t rightTurnSpeedCounts;
    int16_t rightTurnNearSpeedCounts;
    uint32_t turnMinEncoderGapCounts;
    uint16_t turnApproachMs;
    int16_t turnExitOuterSpeedCounts;
    int16_t turnExitInnerSpeedCounts;
    uint16_t turnExitMs;
    uint16_t turnHoldMs;
    uint16_t lineLostTimeoutMs;
} MotorNoYawConfig;

static const MotorNoYawConfig g_motorNoYawConfigs[MOTOR_NO_YAW_PROFILE_COUNT] = {
    {
        {
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
            CAR_MOTOR_NO_YAW_TASK1_PID_S1_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK1_PID_S2_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK1_PID_S3_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK1_PID_S4_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK1_PID_S5_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK1_PID_S6_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK1_PID_S7_WEIGHT
#else
            CAR_MOTOR_NO_YAW_TASK1_PWM_S1_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK1_PWM_S2_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK1_PWM_S3_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK1_PWM_S4_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK1_PWM_S5_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK1_PWM_S6_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK1_PWM_S7_WEIGHT
#endif
        },
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_PID_LINE_DEADBAND,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_PID_BASE_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_PID_MIN_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_PID_MAX_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_PID_GRAY_P_GAIN,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_PID_GRAY_D_GAIN,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK1_PID_CORRECTION_LIMIT_COUNTS,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_PID_SEARCH_SPEED_COUNTS_PER_PERIOD,
#else
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_PWM_LINE_DEADBAND,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_PWM_BASE_COUNTS,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_PWM_SEARCH_COUNTS,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_PWM_GRAY_GAIN,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK1_PWM_GRAY_LIMIT_COUNTS,
        (int32_t)CAR_MOTOR_NO_YAW_TASK1_PWM_SYNC_GAIN_Q1024,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK1_PWM_SYNC_LIMIT_COUNTS,
#endif
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_TURN_INNER_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD,
        (uint32_t)CAR_MOTOR_NO_YAW_TASK1_TURN_MIN_ENCODER_GAP_COUNTS,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK1_TURN_APPROACH_MS,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK1_TURN_EXIT_MS,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK1_TURN_HOLD_MS,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK1_LINE_LOST_TIMEOUT_MS
    },
    {
        {
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
            CAR_MOTOR_NO_YAW_TASK4_PID_S1_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK4_PID_S2_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK4_PID_S3_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK4_PID_S4_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK4_PID_S5_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK4_PID_S6_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK4_PID_S7_WEIGHT
#else
            CAR_MOTOR_NO_YAW_TASK4_PWM_S1_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK4_PWM_S2_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK4_PWM_S3_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK4_PWM_S4_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK4_PWM_S5_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK4_PWM_S6_WEIGHT,
            CAR_MOTOR_NO_YAW_TASK4_PWM_S7_WEIGHT
#endif
        },
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_PID_LINE_DEADBAND,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_PID_BASE_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_PID_MIN_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_PID_MAX_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_PID_GRAY_P_GAIN,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_PID_GRAY_D_GAIN,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK4_PID_CORRECTION_LIMIT_COUNTS,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_PID_SEARCH_SPEED_COUNTS_PER_PERIOD,
#else
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_PWM_LINE_DEADBAND,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_PWM_BASE_COUNTS,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_PWM_SEARCH_COUNTS,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_PWM_GRAY_GAIN,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK4_PWM_GRAY_LIMIT_COUNTS,
        (int32_t)CAR_MOTOR_NO_YAW_TASK4_PWM_SYNC_GAIN_Q1024,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK4_PWM_SYNC_LIMIT_COUNTS,
#endif
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_TURN_INNER_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_LEFT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_RIGHT_TURN_NEAR_SPEED_COUNTS_PER_PERIOD,
        (uint32_t)CAR_MOTOR_NO_YAW_TASK4_TURN_MIN_ENCODER_GAP_COUNTS,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK4_TURN_APPROACH_MS,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_MS,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK4_TURN_HOLD_MS,
        (uint16_t)CAR_MOTOR_NO_YAW_TASK4_LINE_LOST_TIMEOUT_MS
    }
};

typedef struct {
    int16_t lineError;
    int16_t lastLineError;
    int16_t lastLeftTargetCounts;
    int16_t lastRightTargetCounts;
    int16_t lastLeftPwmCounts;
    int16_t lastRightPwmCounts;
    uint16_t turnTicks;
    uint16_t lineLostTicks;
    volatile uint16_t s1RecentSamples;
    volatile uint16_t s2RecentSamples;
    volatile uint16_t s6RecentSamples;
    volatile uint16_t s7RecentSamples;
    volatile uint16_t turnReleaseSamples;
    int32_t turnEncoderLeftBaseCounts;
    int32_t turnEncoderRightBaseCounts;
    int16_t turnYawBaseX100;
    uint16_t turnYawDeltaX100;
    uint32_t turnYawFrameCount;
    uint32_t turnCount;
    uint8_t phase;
    uint8_t hasTurnEncoderBase;
    MotorNoYawProfile profile;
    MotorNoYawTurnDirection turnDirection;
    volatile uint8_t digitalMask;
    volatile uint8_t lineMaskCandidate;
    volatile uint8_t lineMaskConfirmSamples;
    volatile uint8_t lineMaskFilterReady;
    uint8_t lineDerivativeReady;
    uint8_t hasLastLineCommand;
    uint8_t hasTurnYawBase;
    volatile uint8_t turnFlag;
    volatile uint8_t returnFlag;
    volatile uint8_t returnConfirmSamples;
    volatile uint8_t turnRequest;
    volatile uint8_t returnLineRequest;
    volatile uint8_t running;
    volatile MotorNoYawState state;
    volatile MotorNoYawStopReason stopReason;
} MotorNoYawControl;

static MotorNoYawControl g_motorNoYaw;

static void MotorNoYaw_StopWithReason(MotorNoYawStopReason reason);
static void MotorNoYaw_StartTurn(void);

/* 灰度快采样定时器只在正式循迹期间运行，Task8 不承担这 10kHz 中断。 */
static void MotorNoYaw_SetSampleTimerEnabled(uint8_t enabled)
{
    DL_TimerG_stopCounter(GRAY_SAMPLE_TIMER_INST);
    NVIC_ClearPendingIRQ(GRAY_SAMPLE_TIMER_INST_INT_IRQN);
    if (enabled != 0U) {
        DL_TimerG_setTimerCount(GRAY_SAMPLE_TIMER_INST,
            GRAY_SAMPLE_TIMER_LOAD_VALUE);
        NVIC_EnableIRQ(GRAY_SAMPLE_TIMER_INST_INT_IRQN);
        DL_TimerG_startCounter(GRAY_SAMPLE_TIMER_INST);
    }
}

static const MotorNoYawConfig *MotorNoYaw_GetConfig(void)
{
    if ((uint32_t)g_motorNoYaw.profile >=
        (uint32_t)MOTOR_NO_YAW_PROFILE_COUNT) {
        return &g_motorNoYawConfigs[MOTOR_NO_YAW_PROFILE_TASK1];   //防止数组越界，超过指定参数的config都默认指向task1
    }
    return &g_motorNoYawConfigs[g_motorNoYaw.profile];
}

/* 作用：把毫秒换成当前10ms底盘控制周期数。 */
static uint16_t MotorNoYaw_MsToTicks(uint16_t timeMs)
{
    return (uint16_t)((timeMs + CHASSIS_CONTROL_PERIOD_MS - 1U) /
        CHASSIS_CONTROL_PERIOD_MS);
}

/* 把跨越正负180度的JY61航向差换算为0..180度的绝对最短角差。 */
static uint16_t MotorNoYaw_AbsYawDeltaX100(int16_t yawX100,
    int16_t baseYawX100)
{
    int32_t delta = (int32_t)yawX100 - (int32_t)baseYawX100;

    if (delta > 18000L) {
        delta -= 36000L;
    } else if (delta < -18000L) {
        delta += 36000L;
    }
    if (delta < 0) {
        delta = -delta;
    }
    return (uint16_t)delta;
}

/* 在灰度确认入弯时锁存一次航向，后续整个直角转向都不再移动基准。 */
static uint8_t MotorNoYaw_CaptureTurnYawBase(void)
{
    JY61P_Attitude attitude;

    if (JY61P_GetAttitude(&attitude) == 0U) {
        g_motorNoYaw.hasTurnYawBase = 0U;
        return 0U;
    }
    g_motorNoYaw.turnYawBaseX100 = attitude.yawX100;
    g_motorNoYaw.turnYawDeltaX100 = 0U;
    g_motorNoYaw.turnYawFrameCount = attitude.angleFrameCount;
    g_motorNoYaw.hasTurnYawBase = 1U;
    return 1U;
}

/* JY61缺失或没有新帧时保持现状，灰度回线逻辑仍继续工作。 */
static void MotorNoYaw_UpdateTurnYaw(void)
{
    JY61P_Attitude attitude;
    uint16_t deltaX100;

    if ((g_motorNoYaw.hasTurnYawBase == 0U) ||
        (JY61P_GetAttitude(&attitude) == 0U)) {
        return;
    }
    if (attitude.angleFrameCount == g_motorNoYaw.turnYawFrameCount) {
        return;
    }
    g_motorNoYaw.turnYawFrameCount = attitude.angleFrameCount;
    deltaX100 = MotorNoYaw_AbsYawDeltaX100(attitude.yawX100,
        g_motorNoYaw.turnYawBaseX100);
    if (deltaX100 > g_motorNoYaw.turnYawDeltaX100) {
        g_motorNoYaw.turnYawDeltaX100 = deltaX100;
    }
}

/* 按已完成转角把外轮从初始速度线性降到粗略参考角对应的速度。 */
static int16_t MotorNoYaw_CalculateTurnOuterSpeed(
    const MotorNoYawConfig *config, MotorNoYawTurnDirection direction)
{
    int16_t startSpeed = (direction == MOTOR_NO_YAW_TURN_LEFT) ?
        config->leftTurnSpeedCounts : config->rightTurnSpeedCounts;
    int16_t nearSpeed = (direction == MOTOR_NO_YAW_TURN_LEFT) ?
        config->leftTurnNearSpeedCounts : config->rightTurnNearSpeedCounts;
    int32_t startMagnitude = (startSpeed < 0) ?
        -(int32_t)startSpeed : startSpeed;
    int32_t nearMagnitude = (nearSpeed < 0) ?
        -(int32_t)nearSpeed : nearSpeed;
    int32_t deltaX100 = g_motorNoYaw.turnYawDeltaX100;
    int32_t magnitude;

    if (deltaX100 > (int32_t)CAR_MOTOR_NO_YAW_TURN_TARGET_ANGLE_X100) {
        deltaX100 = (int32_t)CAR_MOTOR_NO_YAW_TURN_TARGET_ANGLE_X100;
    }
    if (startMagnitude <= nearMagnitude) {
        magnitude = nearMagnitude;
    } else {
        magnitude = startMagnitude -
            (((startMagnitude - nearMagnitude) * deltaX100 +
                (int32_t)CAR_MOTOR_NO_YAW_TURN_TARGET_ANGLE_X100 / 2) /
                (int32_t)CAR_MOTOR_NO_YAW_TURN_TARGET_ANGLE_X100);
    }
    return (startSpeed < 0) ?
        (int16_t)-magnitude : (int16_t)magnitude;
}

static uint32_t MotorNoYaw_AbsEncoderDelta(int32_t now, int32_t base)
{
    return (now >= base) ? (uint32_t)(now - base) :
        (uint32_t)(base - now);
}

#if CAR_MOTOR_NO_YAW_USE_SPEED_PID

/* 作用：把速度限制在普通循迹允许范围，普通循迹不允许单轮停车。 */
static int16_t MotorNoYaw_ClampLineSpeed(
    const MotorNoYawConfig *config, int32_t speed)
{
    if (speed < (int32_t)config->minLineSpeedCounts) {
        return (int16_t)config->minLineSpeedCounts;
    }
    if (speed > (int32_t)config->maxLineSpeedCounts) {
        return (int16_t)config->maxLineSpeedCounts;
    }
    return (int16_t)speed;
}

/* 作用：限制差速修正，防止普通循迹变成原地强转。 */
static int16_t MotorNoYaw_ClampCorrection(
    const MotorNoYawConfig *config, int32_t correction)
{
    if (correction > (int32_t)config->turnLimitCounts) {
        return (int16_t)config->turnLimitCounts;
    }
    if (correction < -(int32_t)config->turnLimitCounts) {
        return (int16_t)(-(int32_t)config->turnLimitCounts);
    }
    return (int16_t)correction;
}

#else

static int16_t MotorNoYaw_ClampDirectPwm(int32_t pwm)
{
    if (pwm > (int32_t)CHASSIS_PWM_LIMIT_COUNTS) {
        return (int16_t)CHASSIS_PWM_LIMIT_COUNTS;
    }
    if (pwm < 0) {
        return 0;
    }
    return (int16_t)pwm;
}

#endif

/* 作用：吃掉很小的偏差，车在中间附近时不要来回抖。 */
static int16_t MotorNoYaw_ApplyLineDeadband(
    const MotorNoYawConfig *config, int16_t error)
{
    int32_t adjusted = error;
    int32_t deadband = (int32_t)config->lineDeadband;

    if ((adjusted > -deadband) && (adjusted < deadband)) {
        return 0;
    }
    if (adjusted > 0) {
        adjusted -= deadband;
    } else if (adjusted < 0) {
        adjusted += deadband;
    }

    return (int16_t)adjusted;
}

/* 作用：同一完整mask连续出现指定次数后，才交给普通循迹控制。 */
static void MotorNoYaw_UpdateFilteredLineMask(uint8_t mask)
{
    if (g_motorNoYaw.lineMaskFilterReady == 0U) {
        g_motorNoYaw.lineMaskCandidate = mask;
        g_motorNoYaw.lineMaskConfirmSamples = 1U;
        g_motorNoYaw.lineMaskFilterReady = 1U;
    } else if (mask == g_motorNoYaw.digitalMask) {
        g_motorNoYaw.lineMaskCandidate = mask;
        g_motorNoYaw.lineMaskConfirmSamples = 0U;
        return;
    } else if (mask == g_motorNoYaw.lineMaskCandidate) {
        if (g_motorNoYaw.lineMaskConfirmSamples <
            (uint8_t)CAR_MOTOR_NO_YAW_LINE_CONFIRM_SAMPLES) {
            ++g_motorNoYaw.lineMaskConfirmSamples;
        }
    } else {
        g_motorNoYaw.lineMaskCandidate = mask;
        g_motorNoYaw.lineMaskConfirmSamples = 1U;
    }

    if (g_motorNoYaw.lineMaskConfirmSamples >=
        (uint8_t)CAR_MOTOR_NO_YAW_LINE_CONFIRM_SAMPLES) {
        g_motorNoYaw.digitalMask = mask;
        g_motorNoYaw.lineMaskConfirmSamples = 0U;
    }
}

static uint32_t MotorNoYaw_GetTurnEncoderGapCounts(void)
{
    uint32_t leftCounts = MotorNoYaw_AbsEncoderDelta(
        Motor_GetStepCount(MOTOR_CHASSIS_LEFT),
        g_motorNoYaw.turnEncoderLeftBaseCounts);
    uint32_t rightCounts = MotorNoYaw_AbsEncoderDelta(
        Motor_GetStepCount(MOTOR_CHASSIS_RIGHT),
        g_motorNoYaw.turnEncoderRightBaseCounts);

    return (leftCounts + rightCounts) / 2U;
}

/* 记录当前左右累计编码器 count，作为下一次强转间隔的距离基准。 */
static void MotorNoYaw_ResetTurnEncoderGap(void)
{
    g_motorNoYaw.turnEncoderLeftBaseCounts =
        Motor_GetStepCount(MOTOR_CHASSIS_LEFT);
    g_motorNoYaw.turnEncoderRightBaseCounts =
        Motor_GetStepCount(MOTOR_CHASSIS_RIGHT);
    g_motorNoYaw.hasTurnEncoderBase = 1U;
}

static uint8_t MotorNoYaw_CanStartTurn(void)
{
    if (g_motorNoYaw.hasTurnEncoderBase == 0U) {
        return 1U;
    }

    return (uint8_t)(MotorNoYaw_GetTurnEncoderGapCounts() >=
        MotorNoYaw_GetConfig()->turnMinEncoderGapCounts);
}

/*
 * 作用：按数字量算一个很直白的偏差。
 * 说明：S1/S7 权重最大，S2/S6 次之，S3/S5 小修，S4 是中心。
 */
static uint8_t MotorNoYaw_CalcDigitalLineError(
    const MotorNoYawConfig *config, uint8_t mask, int16_t *error)
{
    int16_t sum = 0;
    uint8_t count = 0U;
    uint8_t bit;
    uint32_t i;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        bit = (uint8_t)(1U << ((GRAY_SENSOR_COUNT - 1U) - i));
        if ((mask & bit) != 0U) {
            sum = (int16_t)(sum + config->lineWeight[i]);
            ++count;
        }
    }

    if (count == 0U) {    //没有检测到一个黑线，说明丢线了
        if (error != 0) {    //此时error是指针，说明这个是有效地址
            *error = 0;     //将值置0
        }
        return 0U;    //告诉上层这个数据无效
    }

    if (error != 0) {
        *error = (int16_t)(((int32_t)sum *
            (int32_t)GRAY_LINE_ERROR_SCALE) / (int32_t)count);
    }
    return 1U;
}

#if CAR_MOTOR_NO_YAW_USE_SPEED_PID

static void MotorNoYaw_CalculateLineTargets(const MotorNoYawConfig *config,
    int16_t error, int16_t errorDelta, int16_t *leftTargetCounts,
    int16_t *rightTargetCounts)
{
    int16_t lineError = MotorNoYaw_ApplyLineDeadband(config, error);
    int16_t correction = MotorNoYaw_ClampCorrection(config,
        (((int32_t)lineError * (int32_t)config->turnGain) +
            ((int32_t)errorDelta * (int32_t)config->turnDGain)) /
            (int32_t)GRAY_LINE_ERROR_SCALE);
    int16_t baseSpeed = MotorNoYaw_ClampLineSpeed(config,
        (int32_t)config->baseSpeedCounts);

    *leftTargetCounts = MotorNoYaw_ClampLineSpeed(config,
        (int32_t)baseSpeed - (int32_t)correction);
    *rightTargetCounts = MotorNoYaw_ClampLineSpeed(config,
        (int32_t)baseSpeed + (int32_t)correction);
}

#else

/* 直接PWM方案只保留灰度P修正，增益1表示权重值直接映射到raw PWM。 */
static void MotorNoYaw_CalculateLinePwm(const MotorNoYawConfig *config,
    int16_t error, int16_t *leftPwm, int16_t *rightPwm)
{
    int16_t lineError = MotorNoYaw_ApplyLineDeadband(config, error);
    int32_t correction = ((int32_t)lineError *
        (int32_t)config->grayGain) /
        (int32_t)GRAY_LINE_ERROR_SCALE;

    if (correction > (int32_t)config->grayLimitPwmCounts) {
        correction = (int32_t)config->grayLimitPwmCounts;
    } else if (correction < -(int32_t)config->grayLimitPwmCounts) {
        correction = -(int32_t)config->grayLimitPwmCounts;
    }
    *leftPwm = MotorNoYaw_ClampDirectPwm(
        (int32_t)config->basePwmCounts - correction);
    *rightPwm = MotorNoYaw_ClampDirectPwm(
        (int32_t)config->basePwmCounts + correction);
}

/* 总开关和S4门控同时满足时才返回编码器交叉同步增益。 */
static int32_t MotorNoYaw_GetCrossSyncGainQ1024(
    const MotorNoYawConfig *config, uint8_t digitalMask)
{
#if CAR_MOTOR_NO_YAW_ENABLE_CROSS_SYNC
    return ((digitalMask & MOTOR_NO_YAW_S4_MASK) != 0U) ?
        config->syncGainQ1024 : 0L;
#else
    (void)config;
    (void)digitalMask;
    return 0L;
#endif
}

#endif

uint8_t MotorNoYaw_ApplyTask1LineCommand(uint8_t digitalMask)
{
    const MotorNoYawConfig *config =
        &g_motorNoYawConfigs[MOTOR_NO_YAW_PROFILE_TASK1];
    int16_t error;
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
    int16_t leftTargetCounts;
    int16_t rightTargetCounts;
#else
    int16_t leftPwm;
    int16_t rightPwm;
#endif

    Motor_SetChassisZeroTargetBrake(0U, 0U);
    if (MotorNoYaw_CalcDigitalLineError(config, digitalMask, &error) == 0U) {
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
        Motion_SetChassisPeriodCommand(0, 0);
#else
        Motor_SetChassisCrossCoupledPwm(0, 0, 0L,
            config->syncLimitPwmCounts);
#endif
        return 0U;
    }
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
    MotorNoYaw_CalculateLineTargets(
        config, error, 0, &leftTargetCounts, &rightTargetCounts);
    Motion_SetChassisPeriodCommand(leftTargetCounts, rightTargetCounts);
#else
    MotorNoYaw_CalculateLinePwm(config, error, &leftPwm, &rightPwm);
    Motor_SetChassisCrossCoupledPwm(leftPwm, rightPwm,
        MotorNoYaw_GetCrossSyncGainQ1024(config, digitalMask),
        config->syncLimitPwmCounts);
#endif
    return 1U;
}

/* 作用：普通循迹输出差速，方向沿用原来的 NO YAW 公式。 */
static void MotorNoYaw_ApplyLineControl(void)
{
    const MotorNoYawConfig *config = MotorNoYaw_GetConfig();
    int16_t error = 0;
    int16_t errorDelta = 0;
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
    int16_t leftSpeed;
    int16_t rightSpeed;
#else
    int16_t leftPwm;
    int16_t rightPwm;
#endif

    if (MotorNoYaw_CalcDigitalLineError(config,
        g_motorNoYaw.digitalMask, &error) != 0U) {
        if (g_motorNoYaw.lineDerivativeReady != 0U) {
            errorDelta = (int16_t)(error - g_motorNoYaw.lastLineError);
        } else {
            g_motorNoYaw.lineDerivativeReady = 1U;
        }
        g_motorNoYaw.lastLineError = error;
        g_motorNoYaw.lineError = error;
    }
    Motor_SetChassisZeroTargetBrake(0U, 0U);
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
    MotorNoYaw_CalculateLineTargets(config, g_motorNoYaw.lineError,
        errorDelta, &leftSpeed, &rightSpeed);
    g_motorNoYaw.lastLeftTargetCounts = leftSpeed;
    g_motorNoYaw.lastRightTargetCounts = rightSpeed;
    Motion_SetChassisPeriodCommand(leftSpeed, rightSpeed);
#else
    (void)errorDelta;
    MotorNoYaw_CalculateLinePwm(config, g_motorNoYaw.lineError,
        &leftPwm, &rightPwm);
    g_motorNoYaw.lastLeftPwmCounts = leftPwm;
    g_motorNoYaw.lastRightPwmCounts = rightPwm;
    Motor_SetChassisCrossCoupledPwm(leftPwm, rightPwm,
        MotorNoYaw_GetCrossSyncGainQ1024(config,
            g_motorNoYaw.digitalMask), config->syncLimitPwmCounts);
#endif
    g_motorNoYaw.hasLastLineCommand = 1U;
}

/* 作用：短时间丢线时先沿用上一拍，真丢线再按任务配置温和搜线。 */
static void MotorNoYaw_ApplyLineLostCommand(void)
{
    const MotorNoYawConfig *config = MotorNoYaw_GetConfig();

    Motor_SetChassisZeroTargetBrake(0U, 0U);
    if (g_motorNoYaw.hasLastLineCommand != 0U) {
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
        Motion_SetChassisPeriodCommand(g_motorNoYaw.lastLeftTargetCounts,
            g_motorNoYaw.lastRightTargetCounts);
#else
        Motor_SetChassisCrossCoupledPwm(g_motorNoYaw.lastLeftPwmCounts,
            g_motorNoYaw.lastRightPwmCounts,
            MotorNoYaw_GetCrossSyncGainQ1024(config,
                g_motorNoYaw.digitalMask), config->syncLimitPwmCounts);
#endif
        return;
    }

#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
    Motion_SetChassisPeriodCommand(config->defaultSearchSpeedCounts, 0);
#else
    Motor_SetChassisCrossCoupledPwm(config->searchPwmCounts, 0,
        MotorNoYaw_GetCrossSyncGainQ1024(config,
            g_motorNoYaw.digitalMask), config->syncLimitPwmCounts);
#endif
}

/* 作用：异常停车，OLED 保留在 NO YAW 页面，方便看 mask/err/state。 */
static void MotorNoYaw_StopWithReason(MotorNoYawStopReason reason)
{
    Motor_SetChassisZeroTargetBrake(0U, 0U);
    Motion_SetChassisPeriodCommand(0, 0);
    MotorEnable_SetChassis(0U);
    g_motorNoYaw.running = 0U;
    MotorNoYaw_SetSampleTimerEnabled(0U);
    g_motorNoYaw.stopReason = reason;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_STOP;
}

static void MotorNoYaw_ResetTurnSampleHistory(void)
{
    g_motorNoYaw.s1RecentSamples = 0U;
    g_motorNoYaw.s2RecentSamples = 0U;
    g_motorNoYaw.s6RecentSamples = 0U;
    g_motorNoYaw.s7RecentSamples = 0U;
}

static void MotorNoYaw_UpdateRecentSample(uint8_t mask, uint8_t sensorMask,
    volatile uint16_t *recentSamples, uint16_t windowSamples)
{
    if ((mask & sensorMask) != 0U) {
        *recentSamples = windowSamples;
    } else if (*recentSamples > 0U) {
        --(*recentSamples);
    }
}

/* 作用：记录左侧 S1/S2，两路在窗口内都出现就认为是左直角。 */
static uint8_t MotorNoYaw_RecordLeftTurnSample(uint8_t mask)
{
    MotorNoYaw_UpdateRecentSample(mask, MOTOR_NO_YAW_LEFT_TURN_NEAR_MASK,
        &g_motorNoYaw.s2RecentSamples,
        (uint16_t)MOTOR_NO_YAW_LEFT_TURN_WINDOW_SAMPLES);
    MotorNoYaw_UpdateRecentSample(mask, MOTOR_NO_YAW_LEFT_TURN_OUTER_MASK,
        &g_motorNoYaw.s1RecentSamples,
        (uint16_t)MOTOR_NO_YAW_LEFT_TURN_WINDOW_SAMPLES);

    if ((g_motorNoYaw.s1RecentSamples > 0U) &&
        (g_motorNoYaw.s2RecentSamples > 0U)) {
        g_motorNoYaw.s1RecentSamples = 0U;
        g_motorNoYaw.s2RecentSamples = 0U;
        return 1U;
    }
    return 0U;
}

/* 作用：记录右侧 S6/S7，两路在窗口内都出现就认为是右直角。 */
static uint8_t MotorNoYaw_RecordRightTurnSample(uint8_t mask)
{
    MotorNoYaw_UpdateRecentSample(mask, MOTOR_NO_YAW_RIGHT_TURN_NEAR_MASK,
        &g_motorNoYaw.s6RecentSamples,
        (uint16_t)MOTOR_NO_YAW_RIGHT_TURN_WINDOW_SAMPLES);
    MotorNoYaw_UpdateRecentSample(mask, MOTOR_NO_YAW_RIGHT_TURN_OUTER_MASK,
        &g_motorNoYaw.s7RecentSamples,
        (uint16_t)MOTOR_NO_YAW_RIGHT_TURN_WINDOW_SAMPLES);

    if ((g_motorNoYaw.s6RecentSamples > 0U) &&
        (g_motorNoYaw.s7RecentSamples > 0U)) {
        g_motorNoYaw.s6RecentSamples = 0U;
        g_motorNoYaw.s7RecentSamples = 0U;
        return 1U;
    }

    return 0U;
}

/* 作用：左右直角触发后，先让车头继续往直角里走一点。 */
static void MotorNoYaw_StartTurnApproach(MotorNoYawTurnDirection direction)
{
    (void)MotorNoYaw_CaptureTurnYawBase();
    g_motorNoYaw.turnDirection = direction;
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.lineDerivativeReady = 0U;
    MotorNoYaw_ResetTurnSampleHistory();
    g_motorNoYaw.turnReleaseSamples = 0U;
    g_motorNoYaw.turnFlag = 0U;      /* 进弯后不再允许重复触发强转。 */
    g_motorNoYaw.returnFlag = 0U;
    g_motorNoYaw.returnConfirmSamples = 0U;
    g_motorNoYaw.turnRequest = (uint8_t)MOTOR_NO_YAW_TURN_NONE;
    g_motorNoYaw.returnLineRequest = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_TURN_APPROACH;
    if (MotorNoYaw_GetConfig()->turnApproachMs == 0U) {
        MotorNoYaw_StartTurn();
    }
}

/* 作用：按锁存方向写强转目标；内轮使用轻微反向闭环目标。 */
static void MotorNoYaw_ApplyTurnCommand(const MotorNoYawConfig *config)
{
    int16_t outerSpeed = MotorNoYaw_CalculateTurnOuterSpeed(config,
        g_motorNoYaw.turnDirection);
    int16_t innerSpeed = config->turnInnerSpeedCounts;

    Motor_SetChassisZeroTargetBrake(0U, 0U);
    if (g_motorNoYaw.turnDirection == MOTOR_NO_YAW_TURN_LEFT) {
        Motion_SetChassisPeriodCommand(innerSpeed, outerSpeed);
    } else {
        Motion_SetChassisPeriodCommand(outerSpeed, innerSpeed);
    }
}

/* 作用：切换内外轮目标后，进入已锁存方向的强转状态。 */
static void MotorNoYaw_StartTurn(void)
{
    const MotorNoYawConfig *config = MotorNoYaw_GetConfig();

    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.hasLastLineCommand = 0U;
    g_motorNoYaw.turnFlag = 0U;      /* 强转中不准再次进入强转。 */
    g_motorNoYaw.returnFlag = 0U;    /* 必须先等转向侧最外传感器释放。 */
    g_motorNoYaw.returnConfirmSamples = 0U;
    g_motorNoYaw.turnRequest = (uint8_t)MOTOR_NO_YAW_TURN_NONE;
    g_motorNoYaw.returnLineRequest = 0U;
    MotorNoYaw_ApplyTurnCommand(config);
    g_motorNoYaw.state = (g_motorNoYaw.turnDirection ==
        MOTOR_NO_YAW_TURN_LEFT) ? MOTOR_NO_YAW_STATE_TURN_LEFT :
        MOTOR_NO_YAW_STATE_TURN_RIGHT;
}

static void MotorNoYaw_ApplyTurnExitCommand(const MotorNoYawConfig *config)
{
    int16_t leftSpeed;
    int16_t rightSpeed;

    Motor_SetChassisZeroTargetBrake(0U, 0U);
    if (g_motorNoYaw.turnDirection == MOTOR_NO_YAW_TURN_LEFT) {
        leftSpeed = config->turnExitInnerSpeedCounts;
        rightSpeed = config->turnExitOuterSpeedCounts;
    } else {
        leftSpeed = config->turnExitOuterSpeedCounts;
        rightSpeed = config->turnExitInnerSpeedCounts;
    }
    Motion_SetChassisPeriodCommand(leftSpeed, rightSpeed);
}

/* 作用：转向侧最外灰度重新回线后，按内外轮速度低速出弯。 */
static void MotorNoYaw_StartTurnExit(void)
{
    const MotorNoYawConfig *config = MotorNoYaw_GetConfig();

    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.returnLineRequest = 0U;
    g_motorNoYaw.returnConfirmSamples = 0U;
    MotorNoYaw_ApplyTurnExitCommand(config);
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_TURN_EXIT;
}

/* 作用：低速出弯结束后累计转向次数，并回到普通循迹。 */
static void MotorNoYaw_FinishTurn(void)
{
    ++g_motorNoYaw.turnCount;   //转向次数计数
    g_motorNoYaw.phase = (uint8_t)((g_motorNoYaw.phase + 1U) & 0x03U);
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.hasLastLineCommand = 0U;
    g_motorNoYaw.lineDerivativeReady = 0U;
    MotorNoYaw_ResetTurnSampleHistory();
    g_motorNoYaw.turnReleaseSamples = 0U;
    g_motorNoYaw.turnFlag = 0U;      /* 两侧直角传感器稳定释放后才重新允许强转。 */
    g_motorNoYaw.returnFlag = 0U;
    g_motorNoYaw.returnConfirmSamples = 0U;
    g_motorNoYaw.turnRequest = (uint8_t)MOTOR_NO_YAW_TURN_NONE;
    g_motorNoYaw.returnLineRequest = 0U;
    g_motorNoYaw.hasTurnYawBase = 0U;
    g_motorNoYaw.turnYawDeltaX100 = 0U;
    g_motorNoYaw.turnDirection = MOTOR_NO_YAW_TURN_NONE;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_LINE;
}

/*
 * 作用：TIMG0 中断里的灰度快采样。
 * 使用场景：TIMG0 灰度采样中断调用，本工程当前周期为100us。
 * 说明：中断里只读 GPIO 和置请求标志，不直接控制电机、不打印、不刷屏。
 */
uint8_t MotorNoYaw_TimerSample(void)
{
    static uint16_t sampleDivTicks;
    uint8_t mask;
    uint8_t leftTurnDetected;
    uint8_t rightTurnDetected;
    uint8_t returnMask;
    MotorNoYawState state;

    if (g_motorNoYaw.running == 0U) {
        sampleDivTicks = 0U;
        return 0U;
    }

    ++sampleDivTicks;
    if (sampleDivTicks < (uint16_t)MOTOR_NO_YAW_TIMER_DIV_TICKS) {
        return 0U;
    }
    sampleDivTicks = 0U;

    mask = Gray_ReadDigitalMaskFast();
    MotorNoYaw_UpdateFilteredLineMask(mask);
    state = g_motorNoYaw.state;

    if (state == MOTOR_NO_YAW_STATE_LINE) {  //检测强转
        if (g_motorNoYaw.turnFlag == 0U) {
            if ((mask & MOTOR_NO_YAW_ALL_TURN_MASK) == 0U) {
                if (g_motorNoYaw.turnReleaseSamples <
                    (uint16_t)MOTOR_NO_YAW_TURN_REARM_SAMPLES) {
                    ++g_motorNoYaw.turnReleaseSamples;
                }
                if (g_motorNoYaw.turnReleaseSamples >=
                    (uint16_t)MOTOR_NO_YAW_TURN_REARM_SAMPLES) {
                    g_motorNoYaw.turnReleaseSamples = 0U;
                    g_motorNoYaw.turnFlag = 1U;
                }
            } else {
                g_motorNoYaw.turnReleaseSamples = 0U;
            }
            return 0U;
        }
        leftTurnDetected = MotorNoYaw_RecordLeftTurnSample(mask);
        rightTurnDetected = MotorNoYaw_RecordRightTurnSample(mask);
        if ((leftTurnDetected != rightTurnDetected) &&
            (g_motorNoYaw.turnRequest ==
                (uint8_t)MOTOR_NO_YAW_TURN_NONE)) {
            g_motorNoYaw.turnRequest = (leftTurnDetected != 0U) ?
                (uint8_t)MOTOR_NO_YAW_TURN_LEFT :
                (uint8_t)MOTOR_NO_YAW_TURN_RIGHT;
            return 1U;
        }
        return 0U;
    }

    if ((state == MOTOR_NO_YAW_STATE_TURN_LEFT) ||
        (state == MOTOR_NO_YAW_STATE_TURN_RIGHT)) {
        returnMask = (state == MOTOR_NO_YAW_STATE_TURN_LEFT) ?
            MOTOR_NO_YAW_LEFT_RETURN_MASK : MOTOR_NO_YAW_RIGHT_RETURN_MASK;
        if (g_motorNoYaw.returnFlag == 0U) {
            g_motorNoYaw.returnConfirmSamples = 0U;
            if ((mask & returnMask) == 0U) {
                g_motorNoYaw.returnFlag = 1U;
            }
            return 0U;
        }
        if ((mask & returnMask) == 0U) {
            g_motorNoYaw.returnConfirmSamples = 0U;
            return 0U;
        }
        if (g_motorNoYaw.returnConfirmSamples <
            (uint8_t)CAR_MOTOR_NO_YAW_RETURN_CONFIRM_SAMPLES) {
            ++g_motorNoYaw.returnConfirmSamples;
        }
        if ((g_motorNoYaw.returnConfirmSamples >=
            (uint8_t)CAR_MOTOR_NO_YAW_RETURN_CONFIRM_SAMPLES) &&
            (g_motorNoYaw.returnLineRequest == 0U)) {
            g_motorNoYaw.returnLineRequest = 1U;
            return 1U;
        }
    }

    return 0U;
}

/* 作用：正常循迹，每个10ms底盘控制周期读一次灰度数字量。 */
static void MotorNoYaw_TaskLine(void)
{
    uint16_t lineLostTimeoutTicks =
        MotorNoYaw_MsToTicks(MotorNoYaw_GetConfig()->lineLostTimeoutMs);
    MotorNoYawTurnDirection direction =
        (MotorNoYawTurnDirection)g_motorNoYaw.turnRequest;

    if (direction != MOTOR_NO_YAW_TURN_NONE) {
        g_motorNoYaw.turnRequest = (uint8_t)MOTOR_NO_YAW_TURN_NONE;
        if (MotorNoYaw_CanStartTurn() == 0U) {
            MotorNoYaw_ApplyLineControl();
            return;
        }
        MotorNoYaw_ResetTurnEncoderGap();
        MotorNoYaw_StartTurnApproach(direction);
        return;
    }

    if (g_motorNoYaw.digitalMask == 0U) {
        g_motorNoYaw.lineDerivativeReady = 0U;
        MotorNoYaw_ApplyLineLostCommand();
        if ((lineLostTimeoutTicks != 0U) &&
            (g_motorNoYaw.lineLostTicks >= lineLostTimeoutTicks)) {
            MotorNoYaw_StopWithReason(MOTOR_NO_YAW_STOP_LINE_LOST);
            return;
        }
        ++g_motorNoYaw.lineLostTicks;
        return;
    }

    g_motorNoYaw.lineLostTicks = 0U;
    MotorNoYaw_ApplyLineControl();
}

/* 作用：直角触发后先前进一小段，再按锁存方向强转。 */
static void MotorNoYaw_TaskTurnApproach(void)
{
    uint16_t approachTicks =
        MotorNoYaw_MsToTicks(MotorNoYaw_GetConfig()->turnApproachMs);

    MotorNoYaw_UpdateTurnYaw();
    if (g_motorNoYaw.hasLastLineCommand != 0U) {
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
            Motion_SetChassisPeriodCommand(
                g_motorNoYaw.lastLeftTargetCounts,
                g_motorNoYaw.lastRightTargetCounts);
#else
            Motor_SetChassisCrossCoupledPwm(
                g_motorNoYaw.lastLeftPwmCounts,
                g_motorNoYaw.lastRightPwmCounts,
                MotorNoYaw_GetCrossSyncGainQ1024(MotorNoYaw_GetConfig(),
                    g_motorNoYaw.digitalMask),
                MotorNoYaw_GetConfig()->syncLimitPwmCounts);
#endif
    } else {
#if CAR_MOTOR_NO_YAW_USE_SPEED_PID
            Motion_SetChassisPeriodCommand(
                (int16_t)MotorNoYaw_GetConfig()->baseSpeedCounts,
                (int16_t)MotorNoYaw_GetConfig()->baseSpeedCounts);
#else
            Motor_SetChassisCrossCoupledPwm(
                MotorNoYaw_GetConfig()->basePwmCounts,
                MotorNoYaw_GetConfig()->basePwmCounts,
                MotorNoYaw_GetCrossSyncGainQ1024(MotorNoYaw_GetConfig(),
                    g_motorNoYaw.digitalMask),
                MotorNoYaw_GetConfig()->syncLimitPwmCounts);
#endif
    }

    if (g_motorNoYaw.turnTicks < MOTOR_NO_YAW_INVALID_TICKS) {
        ++g_motorNoYaw.turnTicks;
    }
    if (g_motorNoYaw.turnTicks >= approachTicks) {
        MotorNoYaw_StartTurn();
    }
}

/* 作用：内轮停车、外轮转向；优先响应灰度回线，姿态角只辅助减速。 */
static void MotorNoYaw_TaskTurn(void)
{
    const MotorNoYawConfig *config = MotorNoYaw_GetConfig();
    uint16_t timeoutTicks =
        MotorNoYaw_MsToTicks(config->turnHoldMs);

    if (g_motorNoYaw.returnLineRequest != 0U) {
        MotorNoYaw_StartTurnExit();
        return;
    }
    MotorNoYaw_UpdateTurnYaw();
    MotorNoYaw_ApplyTurnCommand(config);
    if (g_motorNoYaw.turnTicks < MOTOR_NO_YAW_INVALID_TICKS) {
        ++g_motorNoYaw.turnTicks;
    }

    if (g_motorNoYaw.turnTicks >= timeoutTicks) {
        MotorNoYaw_StopWithReason(MOTOR_NO_YAW_STOP_TURN_TIMEOUT);
    }
}

/* 作用：强转解除后双轮按内外轮配置低速前进并平滑恢复循迹。 */
static void MotorNoYaw_TaskTurnExit(void)
{
    const MotorNoYawConfig *config = MotorNoYaw_GetConfig();
    uint16_t exitTicks = MotorNoYaw_MsToTicks(config->turnExitMs);

    MotorNoYaw_ApplyTurnExitCommand(config);
    if (g_motorNoYaw.turnTicks < MOTOR_NO_YAW_INVALID_TICKS) {
        ++g_motorNoYaw.turnTicks;
    }
    if (g_motorNoYaw.turnTicks >= exitTicks) {
        MotorNoYaw_FinishTurn();
    }
}

static void MotorNoYaw_ResetControl(void)
{
    g_motorNoYaw.lineError = 0;
    g_motorNoYaw.lastLineError = 0;
    g_motorNoYaw.lastLeftTargetCounts = 0;
    g_motorNoYaw.lastRightTargetCounts = 0;
    g_motorNoYaw.lastLeftPwmCounts = 0;
    g_motorNoYaw.lastRightPwmCounts = 0;
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    MotorNoYaw_ResetTurnSampleHistory();
    g_motorNoYaw.turnReleaseSamples = 0U;
    g_motorNoYaw.turnEncoderLeftBaseCounts = 0;
    g_motorNoYaw.turnEncoderRightBaseCounts = 0;
    g_motorNoYaw.turnYawBaseX100 = 0;
    g_motorNoYaw.turnYawDeltaX100 = 0U;
    g_motorNoYaw.turnYawFrameCount = 0U;
    g_motorNoYaw.turnCount = 0U;
    g_motorNoYaw.phase = 0U;
    g_motorNoYaw.hasTurnEncoderBase = 0U;
    g_motorNoYaw.turnDirection = MOTOR_NO_YAW_TURN_NONE;
    g_motorNoYaw.digitalMask = 0U;
    g_motorNoYaw.lineMaskCandidate = 0U;
    g_motorNoYaw.lineMaskConfirmSamples = 0U;
    g_motorNoYaw.lineMaskFilterReady = 0U;
    g_motorNoYaw.lineDerivativeReady = 0U;
    g_motorNoYaw.hasLastLineCommand = 0U;
    g_motorNoYaw.hasTurnYawBase = 0U;
    g_motorNoYaw.turnFlag = 1U;
    g_motorNoYaw.returnFlag = 0U;
    g_motorNoYaw.returnConfirmSamples = 0U;
    g_motorNoYaw.turnRequest = (uint8_t)MOTOR_NO_YAW_TURN_NONE;
    g_motorNoYaw.returnLineRequest = 0U;
    g_motorNoYaw.stopReason = MOTOR_NO_YAW_STOP_NONE;
}

void MotorNoYaw_Init(void)
{
    g_motorNoYaw.profile = MOTOR_NO_YAW_PROFILE_TASK1;
    MotorNoYaw_ResetControl();
    g_motorNoYaw.running = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_IDLE;
    MotorNoYaw_SetSampleTimerEnabled(0U);
}

static void MotorNoYaw_StartWithProfile(MotorNoYawProfile profile)
{
    Motor_SetChassisZeroTargetBrake(0U, 0U);
    g_motorNoYaw.profile = profile;
    MotorNoYaw_ResetControl();
    MotorNoYaw_UpdateFilteredLineMask(Gray_ReadDigitalMaskFast());
    g_motorNoYaw.running = 1U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_LINE;
    MotorNoYaw_SetSampleTimerEnabled(1U);
    MotorEnable_SetChassis(1U);
}

void MotorNoYaw_Start(void)
{
    MotorNoYaw_StartWithProfile(MOTOR_NO_YAW_PROFILE_TASK1);
}

void MotorNoYaw_StartMission4(void)
{
    MotorNoYaw_StartWithProfile(MOTOR_NO_YAW_PROFILE_TASK4);
}

void MotorNoYaw_Stop(void)
{
    Motor_SetChassisZeroTargetBrake(0U, 0U);
    if (g_motorNoYaw.state != MOTOR_NO_YAW_STATE_IDLE) {
        Motion_SetChassisPeriodCommand(0, 0);
    MotorEnable_SetChassis(0U);
    }
    g_motorNoYaw.running = 0U;
    MotorNoYaw_SetSampleTimerEnabled(0U);
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_IDLE;
}

void MotorNoYaw_HandleFastEvent(void)
{
    MotorNoYawTurnDirection direction;

    if (g_motorNoYaw.running == 0U) {
        return;
    }

    direction = (MotorNoYawTurnDirection)g_motorNoYaw.turnRequest;
    if ((g_motorNoYaw.state == MOTOR_NO_YAW_STATE_LINE) &&
        (direction != MOTOR_NO_YAW_TURN_NONE)) {
        g_motorNoYaw.turnRequest = (uint8_t)MOTOR_NO_YAW_TURN_NONE;
        if (MotorNoYaw_CanStartTurn() != 0U) {
            MotorNoYaw_ResetTurnEncoderGap();
            MotorNoYaw_StartTurnApproach(direction);
        }
        return;
    }

    if (((g_motorNoYaw.state == MOTOR_NO_YAW_STATE_TURN_LEFT) ||
        (g_motorNoYaw.state == MOTOR_NO_YAW_STATE_TURN_RIGHT)) &&
        (g_motorNoYaw.returnLineRequest != 0U)) {
        MotorNoYaw_StartTurnExit();
    }

}

void MotorNoYaw_Task(void)
{
    if (g_motorNoYaw.running == 0U) {
        return;
    }

    switch (g_motorNoYaw.state) {
    case MOTOR_NO_YAW_STATE_LINE:
        MotorNoYaw_TaskLine();
        break;
    case MOTOR_NO_YAW_STATE_TURN_APPROACH:
        MotorNoYaw_TaskTurnApproach();
        break;
    case MOTOR_NO_YAW_STATE_TURN_LEFT:
    case MOTOR_NO_YAW_STATE_TURN_RIGHT:
        MotorNoYaw_TaskTurn();
        break;
    case MOTOR_NO_YAW_STATE_TURN_EXIT:
        MotorNoYaw_TaskTurnExit();
        break;
    case MOTOR_NO_YAW_STATE_STOP:
    case MOTOR_NO_YAW_STATE_IDLE:
    default:
        break;
    }
}

uint8_t MotorNoYaw_IsRunning(void)
{
    return g_motorNoYaw.running;
}

MotorNoYawState MotorNoYaw_GetState(void)
{
    return g_motorNoYaw.state;
}

MotorNoYawStopReason MotorNoYaw_GetStopReason(void)
{
    return g_motorNoYaw.stopReason;
}

const char *MotorNoYaw_GetStateName(void)
{
    switch (g_motorNoYaw.state) {
    case MOTOR_NO_YAW_STATE_LINE:
        return "Line";
    case MOTOR_NO_YAW_STATE_TURN_APPROACH:
        return "Approach";
    case MOTOR_NO_YAW_STATE_TURN_RIGHT:
        return "RTurn";
    case MOTOR_NO_YAW_STATE_TURN_EXIT:
        return "Exit";
    case MOTOR_NO_YAW_STATE_TURN_LEFT:
        return "LTurn";
    case MOTOR_NO_YAW_STATE_STOP:
        return "Stop";
    case MOTOR_NO_YAW_STATE_IDLE:
    default:
        return "Idle";
    }
}

uint8_t MotorNoYaw_GetPhase(void)
{
    return g_motorNoYaw.phase;
}

uint32_t MotorNoYaw_GetTurnCount(void)
{
    return g_motorNoYaw.turnCount;
}

uint8_t MotorNoYaw_GetDigitalMask(void)
{
    return g_motorNoYaw.digitalMask;
}

int16_t MotorNoYaw_GetLineError(void)
{
    return g_motorNoYaw.lineError;
}
