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

#define MOTOR_NO_YAW_RETURN_MASK    MOTOR_NO_YAW_S7_MASK
#define MOTOR_NO_YAW_RIGHT_TURN_MASK \
    (MOTOR_NO_YAW_S5_MASK | MOTOR_NO_YAW_S6_MASK | \
        MOTOR_NO_YAW_S7_MASK)

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

#if ((CAR_MOTOR_NO_YAW_MIN_LINE_SPEED_COUNTS_PER_PERIOD > \
        CAR_MOTOR_NO_YAW_BASE_SPEED_COUNTS_PER_PERIOD) || \
    (CAR_MOTOR_NO_YAW_BASE_SPEED_COUNTS_PER_PERIOD > \
        CAR_MOTOR_NO_YAW_MAX_LINE_SPEED_COUNTS_PER_PERIOD))
#error "Task1 line speeds must satisfy min <= base <= max"
#endif

#if ((MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_MIN_LINE_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_MAX_LINE_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (CAR_MOTOR_NO_YAW_TURN_LIMIT_COUNTS_PER_PERIOD > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TURN_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TURN_NEAR_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_DEFAULT_SEARCH_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (MOTOR_NO_YAW_ABS_TARGET( \
        CAR_MOTOR_NO_YAW_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD))
#error "Task1 speed targets must stay within the Task5 -100..100 scale"
#endif

#if (((CAR_MOTOR_NO_YAW_TASK4_BASE_SPEED_CPS % CHASSIS_CONTROL_HZ) != 0U) || \
    ((CAR_MOTOR_NO_YAW_TASK4_MIN_LINE_SPEED_CPS % CHASSIS_CONTROL_HZ) != 0U) || \
    ((CAR_MOTOR_NO_YAW_TASK4_MAX_LINE_SPEED_CPS % CHASSIS_CONTROL_HZ) != 0U) || \
    ((CAR_MOTOR_NO_YAW_TASK4_TURN_LIMIT_CPS % CHASSIS_CONTROL_HZ) != 0U) || \
    ((CAR_MOTOR_NO_YAW_TASK4_TURN_SPEED_CPS % CHASSIS_CONTROL_HZ) != 0U) || \
    ((CAR_MOTOR_NO_YAW_TASK4_TURN_NEAR_SPEED_CPS % CHASSIS_CONTROL_HZ) != 0U) || \
    ((CAR_MOTOR_NO_YAW_TASK4_DEFAULT_SEARCH_SPEED_CPS % CHASSIS_CONTROL_HZ) != 0U) || \
    ((CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_OUTER_SPEED_CPS % CHASSIS_CONTROL_HZ) != 0U) || \
    ((CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_INNER_SPEED_CPS % CHASSIS_CONTROL_HZ) != 0U))
#error "Task4 CPS speeds must map exactly to count/control-period"
#endif

#define MOTOR_NO_YAW_CPS_TO_PERIOD_COUNTS(value) \
    ((value) / CHASSIS_CONTROL_HZ)

#define MOTOR_NO_YAW_TIMER_DIV_TICKS \
    (((STEPPER_TIMER_TICK_HZ * CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US) + \
        999999U) / 1000000U)

#if (MOTOR_NO_YAW_TIMER_DIV_TICKS == 0U)
#error "MOTOR_NO_YAW_TIMER_DIV_TICKS must be greater than 0"
#endif

#define MOTOR_NO_YAW_RIGHT_TURN_WINDOW_SAMPLES \
    (((CAR_MOTOR_NO_YAW_RIGHT_TURN_WINDOW_MS * 1000U) + \
        CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US - 1U) / \
        CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US)

#define MOTOR_NO_YAW_TURN_REARM_SAMPLES \
    (((CAR_MOTOR_NO_YAW_TURN_REARM_MS * 1000U) + \
        CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US - 1U) / \
        CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US)

#if ((MOTOR_NO_YAW_TURN_REARM_SAMPLES == 0U) || \
    (MOTOR_NO_YAW_TURN_REARM_SAMPLES > 0xFFFFU))
#error "Task1 turn rearm samples must fit uint16_t and be greater than zero"
#endif

typedef enum {
    MOTOR_NO_YAW_PROFILE_TASK1 = 0,
    MOTOR_NO_YAW_PROFILE_TASK4,
    MOTOR_NO_YAW_PROFILE_COUNT
} MotorNoYawProfile;

typedef struct {
    int16_t baseSpeedCounts;
    int16_t minLineSpeedCounts;
    int16_t maxLineSpeedCounts;
    int16_t lineDeadband;
    int16_t turnGain;
    int16_t turnDGain;
    uint16_t turnLimitCounts;
    int16_t turnSpeedCounts;
    int16_t turnNearSpeedCounts;
    uint32_t turnMinEncoderGapCounts;
    int16_t defaultSearchSpeedCounts;
    uint16_t turnApproachMs;
    int16_t turnExitOuterSpeedCounts;
    int16_t turnExitInnerSpeedCounts;
    uint16_t turnExitMs;
    uint16_t turnHoldMs;
    uint16_t lineLostTimeoutMs;
} MotorNoYawConfig;

static const MotorNoYawConfig g_motorNoYawConfigs[MOTOR_NO_YAW_PROFILE_COUNT] = {
    {
        (int16_t)CAR_MOTOR_NO_YAW_BASE_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_MIN_LINE_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_MAX_LINE_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_LINE_DEADBAND,
        (int16_t)CAR_MOTOR_NO_YAW_TURN_GAIN,
        (int16_t)CAR_MOTOR_NO_YAW_TURN_D_GAIN,
        (uint16_t)CAR_MOTOR_NO_YAW_TURN_LIMIT_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TURN_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TURN_NEAR_SPEED_COUNTS_PER_PERIOD,
        (uint32_t)CAR_MOTOR_NO_YAW_TURN_MIN_ENCODER_GAP_COUNTS,
        (int16_t)CAR_MOTOR_NO_YAW_DEFAULT_SEARCH_SPEED_COUNTS_PER_PERIOD,
        (uint16_t)CAR_MOTOR_NO_YAW_TURN_APPROACH_MS,
        (int16_t)CAR_MOTOR_NO_YAW_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD,
        (int16_t)CAR_MOTOR_NO_YAW_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD,
        (uint16_t)CAR_MOTOR_NO_YAW_TURN_EXIT_MS,
        (uint16_t)CAR_MOTOR_NO_YAW_TURN_HOLD_MS,
        (uint16_t)CAR_MOTOR_NO_YAW_LINE_LOST_TIMEOUT_MS
    },
    {
        (int16_t)MOTOR_NO_YAW_CPS_TO_PERIOD_COUNTS(
            CAR_MOTOR_NO_YAW_TASK4_BASE_SPEED_CPS),
        (int16_t)MOTOR_NO_YAW_CPS_TO_PERIOD_COUNTS(
            CAR_MOTOR_NO_YAW_TASK4_MIN_LINE_SPEED_CPS),
        (int16_t)MOTOR_NO_YAW_CPS_TO_PERIOD_COUNTS(
            CAR_MOTOR_NO_YAW_TASK4_MAX_LINE_SPEED_CPS),
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_LINE_DEADBAND,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_TURN_GAIN,
        (int16_t)CAR_MOTOR_NO_YAW_TASK4_TURN_D_GAIN,
        (uint16_t)MOTOR_NO_YAW_CPS_TO_PERIOD_COUNTS(
            CAR_MOTOR_NO_YAW_TASK4_TURN_LIMIT_CPS),
        (int16_t)MOTOR_NO_YAW_CPS_TO_PERIOD_COUNTS(
            CAR_MOTOR_NO_YAW_TASK4_TURN_SPEED_CPS),
        (int16_t)MOTOR_NO_YAW_CPS_TO_PERIOD_COUNTS(
            CAR_MOTOR_NO_YAW_TASK4_TURN_NEAR_SPEED_CPS),
        (uint32_t)CAR_MOTOR_NO_YAW_TASK4_TURN_MIN_ENCODER_GAP_COUNTS,
        (int16_t)MOTOR_NO_YAW_CPS_TO_PERIOD_COUNTS(
            CAR_MOTOR_NO_YAW_TASK4_DEFAULT_SEARCH_SPEED_CPS),
        (uint16_t)CAR_MOTOR_NO_YAW_TASK4_TURN_APPROACH_MS,
        (int16_t)MOTOR_NO_YAW_CPS_TO_PERIOD_COUNTS(
            CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_OUTER_SPEED_CPS),
        (int16_t)MOTOR_NO_YAW_CPS_TO_PERIOD_COUNTS(
            CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_INNER_SPEED_CPS),
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
    uint16_t turnTicks;
    uint16_t lineLostTicks;
    volatile uint16_t s5RecentSamples;
    volatile uint16_t s6RecentSamples;
    volatile uint16_t s7RecentSamples;
    volatile uint16_t rightTurnReleaseSamples;
    int32_t turnEncoderLeftBaseCounts;
    int32_t turnEncoderRightBaseCounts;
    int16_t turnYawBaseX100;
    uint16_t turnYawDeltaX100;
    uint32_t turnYawFrameCount;
    uint32_t turnCount;
    uint8_t phase;
    uint8_t hasTurnEncoderBase;
    MotorNoYawProfile profile;
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
    volatile uint8_t rightTurnRequest;
    volatile uint8_t returnLineRequest;
    volatile uint8_t running;
    volatile MotorNoYawState state;
} MotorNoYawControl;

static MotorNoYawControl g_motorNoYaw;

static void MotorNoYaw_StopWithReason(const char *reason);

static const MotorNoYawConfig *MotorNoYaw_GetConfig(void)
{
    if ((uint32_t)g_motorNoYaw.profile >=
        (uint32_t)MOTOR_NO_YAW_PROFILE_COUNT) {
        return &g_motorNoYawConfigs[MOTOR_NO_YAW_PROFILE_TASK1];   //防止数组越界，超过指定参数的config都默认指向task1
    }
    return &g_motorNoYawConfigs[g_motorNoYaw.profile];
}

/* 作用：把毫秒换成20ms底盘控制周期数。 */
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

/* 在灰度确认入弯时锁存一次航向，后续整个右转都不再移动基准。 */
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

/* 按已完成转角把强转左轮从初始速度线性降到粗略参考角对应的速度。 */
static int16_t MotorNoYaw_CalculateRightTurnSpeed(
    const MotorNoYawConfig *config)
{
    int32_t startMagnitude = (config->turnSpeedCounts < 0) ?
        -(int32_t)config->turnSpeedCounts : config->turnSpeedCounts;
    int32_t nearMagnitude = (config->turnNearSpeedCounts < 0) ?
        -(int32_t)config->turnNearSpeedCounts : config->turnNearSpeedCounts;
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
    return (config->turnSpeedCounts < 0) ?
        (int16_t)-magnitude : (int16_t)magnitude;
}

static uint32_t MotorNoYaw_AbsEncoderDelta(int32_t now, int32_t base)
{
    return (now >= base) ? (uint32_t)(now - base) :
        (uint32_t)(base - now);
}

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

static uint8_t MotorNoYaw_CanStartRightTurn(void)  //判断：现在能不能开始一次新的右直角强转
{
    if (g_motorNoYaw.hasTurnEncoderBase == 0U) {   //如果还没有记录过右转基准点，说明这是第一次右直角触发，直接允许右转
        return 1U;
    }

    return (uint8_t)(MotorNoYaw_GetTurnEncoderGapCounts() >=
        MotorNoYaw_GetConfig()->turnMinEncoderGapCounts);
}

/*
 * 作用：按数字量算一个很直白的偏差。
 * 说明：S1/S7 权重最大，S2/S6 次之，S3/S5 小修，S4 是中心。
 */
static uint8_t MotorNoYaw_CalcDigitalLineError(uint8_t mask,
    int16_t *error)
{
    static const int16_t weight[GRAY_SENSOR_COUNT] = {   //左右轮不对称，所以灰度设计不对称
        -11, -5, -2,0, 2 ,5,11
    };
    int16_t sum = 0;
    uint8_t count = 0U;
    uint8_t bit;
    uint32_t i;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        bit = (uint8_t)(1U << ((GRAY_SENSOR_COUNT - 1U) - i));
        if ((mask & bit) != 0U) {
            sum = (int16_t)(sum + weight[i]);
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

uint8_t MotorNoYaw_CalculateTask1LineCommand(uint8_t digitalMask,
    int16_t *leftTargetCounts, int16_t *rightTargetCounts)
{
    int16_t error;

    if ((leftTargetCounts == 0) || (rightTargetCounts == 0)) {
        return 0U;
    }
    *leftTargetCounts = 0;
    *rightTargetCounts = 0;
    if (MotorNoYaw_CalcDigitalLineError(digitalMask, &error) == 0U) {
        return 0U;
    }
    MotorNoYaw_CalculateLineTargets(
        &g_motorNoYawConfigs[MOTOR_NO_YAW_PROFILE_TASK1], error, 0,
        leftTargetCounts, rightTargetCounts);
    return 1U;
}

/* 作用：普通循迹输出差速，方向沿用原来的 NO YAW 公式。 */
static void MotorNoYaw_ApplyLineControl(void)
{
    const MotorNoYawConfig *config = MotorNoYaw_GetConfig();
    int16_t error = 0;
    int16_t errorDelta = 0;
    int16_t leftSpeed;
    int16_t rightSpeed;

    if (MotorNoYaw_CalcDigitalLineError(g_motorNoYaw.digitalMask,
        &error) != 0U) {
        if (g_motorNoYaw.lineDerivativeReady != 0U) {
            errorDelta = (int16_t)(error - g_motorNoYaw.lastLineError);
        } else {
            g_motorNoYaw.lineDerivativeReady = 1U;
        }
        g_motorNoYaw.lastLineError = error;
        g_motorNoYaw.lineError = error;
    }
    MotorNoYaw_CalculateLineTargets(config, g_motorNoYaw.lineError,
        errorDelta, &leftSpeed, &rightSpeed);

    g_motorNoYaw.lastLeftTargetCounts = leftSpeed;
    g_motorNoYaw.lastRightTargetCounts = rightSpeed;
    g_motorNoYaw.hasLastLineCommand = 1U;
    Motion_SetChassisPeriodCommand(leftSpeed, rightSpeed);
}

/* 作用：短时间丢线时先沿用上一拍，真丢线再按任务配置温和搜线。 */
static void MotorNoYaw_ApplyLineLostCommand(void)
{
    const MotorNoYawConfig *config = MotorNoYaw_GetConfig();

    if (g_motorNoYaw.hasLastLineCommand != 0U) {
        Motion_SetChassisPeriodCommand(g_motorNoYaw.lastLeftTargetCounts,
            g_motorNoYaw.lastRightTargetCounts);
        return;
    }

    if (g_motorNoYaw.profile == MOTOR_NO_YAW_PROFILE_TASK1) {
        Motion_SetChassisPeriodCommand(config->defaultSearchSpeedCounts, 0);
    } else {
        Motion_SetChassisPeriodCommand(0, config->defaultSearchSpeedCounts);
    }
}

/* 作用：异常停车，OLED 保留在 NO YAW 页面，方便看 mask/err/state。 */
static void MotorNoYaw_StopWithReason(const char *reason)
{
    (void)reason;

    Motion_SetChassisPeriodCommand(0, 0);
    MotorEnable_SetChassis(0U);
    g_motorNoYaw.running = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_STOP;
}

/* 作用：记录 S5/S6/S7 最近是否触发，窗口内三路都出现就认为是右直角。 */
static uint8_t MotorNoYaw_RecordRightTurnSample(uint8_t mask)
{
    if ((mask & MOTOR_NO_YAW_S5_MASK) != 0U) {
        g_motorNoYaw.s5RecentSamples =
            (uint16_t)MOTOR_NO_YAW_RIGHT_TURN_WINDOW_SAMPLES;   //“倒计时计数器
    } else if (g_motorNoYaw.s5RecentSamples > 0U) {
        --g_motorNoYaw.s5RecentSamples;
    }

    if ((mask & MOTOR_NO_YAW_S6_MASK) != 0U) {
        g_motorNoYaw.s6RecentSamples =
            (uint16_t)MOTOR_NO_YAW_RIGHT_TURN_WINDOW_SAMPLES;   //“倒计时计数器
    } else if (g_motorNoYaw.s6RecentSamples > 0U) {
        --g_motorNoYaw.s6RecentSamples;
    }

    if ((mask & MOTOR_NO_YAW_S7_MASK) != 0U) {
        g_motorNoYaw.s7RecentSamples =
            (uint16_t)MOTOR_NO_YAW_RIGHT_TURN_WINDOW_SAMPLES;
    } else if (g_motorNoYaw.s7RecentSamples > 0U) {
        --g_motorNoYaw.s7RecentSamples;
    }

    if ((g_motorNoYaw.s5RecentSamples > 0U) &&
        (g_motorNoYaw.s6RecentSamples > 0U) &&
        (g_motorNoYaw.s7RecentSamples > 0U)) {
        g_motorNoYaw.s5RecentSamples = 0U;
        g_motorNoYaw.s6RecentSamples = 0U;
        g_motorNoYaw.s7RecentSamples = 0U;
        return 1U;
    }

    return 0U;
}

/* 作用：S5/S6/S7 触发后，先让车头继续往直角里走一点。 */
static void MotorNoYaw_StartTurnApproach(void)
{
    (void)MotorNoYaw_CaptureTurnYawBase();
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.lineDerivativeReady = 0U;
    g_motorNoYaw.s5RecentSamples = 0U;
    g_motorNoYaw.s6RecentSamples = 0U;
    g_motorNoYaw.s7RecentSamples = 0U;
    g_motorNoYaw.rightTurnReleaseSamples = 0U;
    g_motorNoYaw.turnFlag = 0U;      /* 进弯后不再允许重复触发强转。 */
    g_motorNoYaw.returnFlag = 0U;
    g_motorNoYaw.returnConfirmSamples = 0U;
    g_motorNoYaw.rightTurnRequest = 0U;
    g_motorNoYaw.returnLineRequest = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_TURN_APPROACH;
}

/* 作用：前进时间到了以后强右转；JY61只提供可选的角度减速参考。 */
static void MotorNoYaw_StartRightTurn(void)
{
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.hasLastLineCommand = 0U;
    g_motorNoYaw.turnFlag = 0U;      /* 强转中不准再次进入强转。 */
    g_motorNoYaw.returnFlag = 0U;    /* 必须先等入弯前的S4释放。 */
    g_motorNoYaw.returnConfirmSamples = 0U;
    g_motorNoYaw.rightTurnRequest = 0U;
    g_motorNoYaw.returnLineRequest = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_TURN_RIGHT;
}

/* 作用：灰度S4重新回线后低速向前出弯。 */
static void MotorNoYaw_StartTurnExit(void)
{
    const MotorNoYawConfig *config = MotorNoYaw_GetConfig();

    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.returnLineRequest = 0U;
    g_motorNoYaw.returnConfirmSamples = 0U;
    Motion_SetChassisPeriodCommand((int16_t)config->turnExitOuterSpeedCounts,
        (int16_t)config->turnExitInnerSpeedCounts);
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_TURN_EXIT;
}

/* 作用：低速出弯结束后累计转向次数，并回到普通循迹。 */
static void MotorNoYaw_FinishRightTurn(void)
{
    ++g_motorNoYaw.turnCount;   //转向次数计数
    g_motorNoYaw.phase = (uint8_t)((g_motorNoYaw.phase + 1U) & 0x03U);
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.hasLastLineCommand = 0U;
    g_motorNoYaw.lineDerivativeReady = 0U;
    g_motorNoYaw.s5RecentSamples = 0U;
    g_motorNoYaw.s6RecentSamples = 0U;
    g_motorNoYaw.s7RecentSamples = 0U;
    g_motorNoYaw.rightTurnReleaseSamples = 0U;
    g_motorNoYaw.turnFlag = 0U;      /* S5/S6/S7 稳定释放后才重新允许强转。 */
    g_motorNoYaw.returnFlag = 0U;
    g_motorNoYaw.returnConfirmSamples = 0U;
    g_motorNoYaw.rightTurnRequest = 0U;
    g_motorNoYaw.returnLineRequest = 0U;
    g_motorNoYaw.hasTurnYawBase = 0U;
    g_motorNoYaw.turnYawDeltaX100 = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_LINE;
}

/*
 * 作用：TIMG0 中断里的灰度快采样。
 * 使用场景：每个云台STEP定时器tick调一次，本函数内部再分频到100us左右。
 * 说明：中断里只读 GPIO 和置请求标志，不直接控制电机、不打印、不刷屏。
 */
uint8_t MotorNoYaw_TimerSample(void)
{
    static uint16_t sampleDivTicks;
    uint8_t mask;
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
            if ((mask & MOTOR_NO_YAW_RIGHT_TURN_MASK) == 0U) {
                if (g_motorNoYaw.rightTurnReleaseSamples <
                    (uint16_t)MOTOR_NO_YAW_TURN_REARM_SAMPLES) {
                    ++g_motorNoYaw.rightTurnReleaseSamples;
                }
                if (g_motorNoYaw.rightTurnReleaseSamples >=
                    (uint16_t)MOTOR_NO_YAW_TURN_REARM_SAMPLES) {
                    g_motorNoYaw.rightTurnReleaseSamples = 0U;
                    g_motorNoYaw.turnFlag = 1U;
                }
            } else {
                g_motorNoYaw.rightTurnReleaseSamples = 0U;
            }
            return 0U;
        }
        if ((g_motorNoYaw.turnFlag != 0U) &&
            (MotorNoYaw_RecordRightTurnSample(mask) != 0U)) {
            if (g_motorNoYaw.rightTurnRequest == 0U) {
                g_motorNoYaw.rightTurnRequest = 1U;
                return 1U;
            }
        }
        return 0U;
    }

    if (state == MOTOR_NO_YAW_STATE_TURN_RIGHT) {
        if (g_motorNoYaw.returnFlag == 0U) {
            g_motorNoYaw.returnConfirmSamples = 0U;
            if ((mask & MOTOR_NO_YAW_RETURN_MASK) == 0U) {
                g_motorNoYaw.returnFlag = 1U;
            }
            return 0U;
        }
        if ((mask & MOTOR_NO_YAW_RETURN_MASK) == 0U) {
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

/* 作用：正常循迹，每个20ms底盘控制周期读一次灰度数字量。 */
static void MotorNoYaw_TaskLine(void)
{
    uint16_t lineLostTimeoutTicks =
        MotorNoYaw_MsToTicks(MotorNoYaw_GetConfig()->lineLostTimeoutMs);

    if (g_motorNoYaw.rightTurnRequest != 0U) {
        g_motorNoYaw.rightTurnRequest = 0U;
        if (MotorNoYaw_CanStartRightTurn() == 0U) {
            MotorNoYaw_ApplyLineControl();
            return;
        }
    MotorNoYaw_ResetTurnEncoderGap();
        MotorNoYaw_StartTurnApproach();
        return;
    }

    if (g_motorNoYaw.digitalMask == 0U) {
        g_motorNoYaw.lineDerivativeReady = 0U;
        MotorNoYaw_ApplyLineLostCommand();
        if ((lineLostTimeoutTicks != 0U) &&
            (g_motorNoYaw.lineLostTicks >= lineLostTimeoutTicks)) {
            MotorNoYaw_StopWithReason("line-lost");
            return;
        }
        ++g_motorNoYaw.lineLostTicks;
        return;
    }

    g_motorNoYaw.lineLostTicks = 0U;
    MotorNoYaw_ApplyLineControl();
}

/* 作用：右直角触发后，先前进一小段，再开始强右转。 */
static void MotorNoYaw_TaskTurnApproach(void)
{
    uint16_t approachTicks =
        MotorNoYaw_MsToTicks(MotorNoYaw_GetConfig()->turnApproachMs);

    MotorNoYaw_UpdateTurnYaw();
    if (g_motorNoYaw.hasLastLineCommand != 0U) {
            Motion_SetChassisPeriodCommand(
                g_motorNoYaw.lastLeftTargetCounts,
                g_motorNoYaw.lastRightTargetCounts);
    } else {
            Motion_SetChassisPeriodCommand(
                (int16_t)MotorNoYaw_GetConfig()->baseSpeedCounts,
                (int16_t)MotorNoYaw_GetConfig()->baseSpeedCounts);
    }

    if (g_motorNoYaw.turnTicks < MOTOR_NO_YAW_INVALID_TICKS) {
        ++g_motorNoYaw.turnTicks;
    }
    if (g_motorNoYaw.turnTicks >= approachTicks) {
        MotorNoYaw_StartRightTurn();
    }
}

/* 作用：右内轮停车；优先响应S4回线，JY61只辅助减速且缺失时忽略。 */
static void MotorNoYaw_TaskTurnRight(void)
{
    const MotorNoYawConfig *config = MotorNoYaw_GetConfig();
    uint16_t timeoutTicks =
        MotorNoYaw_MsToTicks(config->turnHoldMs);
    int16_t leftSpeed;

    if (g_motorNoYaw.returnLineRequest != 0U) {
        MotorNoYaw_StartTurnExit();
        return;
    }
    MotorNoYaw_UpdateTurnYaw();
    leftSpeed = MotorNoYaw_CalculateRightTurnSpeed(config);
    Motion_SetChassisPeriodCommand(leftSpeed, 0);
    if (g_motorNoYaw.turnTicks < MOTOR_NO_YAW_INVALID_TICKS) {
        ++g_motorNoYaw.turnTicks;
    }

    if (g_motorNoYaw.turnTicks >= timeoutTicks) {
        MotorNoYaw_StopWithReason("turn-timeout");
    }
}

/* 作用：强转解除后双轮低速前进，右内轮略快并平滑恢复循迹。 */
static void MotorNoYaw_TaskTurnExit(void)
{
    const MotorNoYawConfig *config = MotorNoYaw_GetConfig();
    uint16_t exitTicks = MotorNoYaw_MsToTicks(config->turnExitMs);

        Motion_SetChassisPeriodCommand(
            (int16_t)config->turnExitOuterSpeedCounts,
            (int16_t)config->turnExitInnerSpeedCounts);
    if (g_motorNoYaw.turnTicks < MOTOR_NO_YAW_INVALID_TICKS) {
        ++g_motorNoYaw.turnTicks;
    }
    if (g_motorNoYaw.turnTicks >= exitTicks) {
        MotorNoYaw_FinishRightTurn();
    }
}

static void MotorNoYaw_ResetControl(void)
{
    g_motorNoYaw.lineError = 0;
    g_motorNoYaw.lastLineError = 0;
    g_motorNoYaw.lastLeftTargetCounts = 0;
    g_motorNoYaw.lastRightTargetCounts = 0;
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.s5RecentSamples = 0U;
    g_motorNoYaw.s6RecentSamples = 0U;
    g_motorNoYaw.s7RecentSamples = 0U;
    g_motorNoYaw.rightTurnReleaseSamples = 0U;
    g_motorNoYaw.turnEncoderLeftBaseCounts = 0;
    g_motorNoYaw.turnEncoderRightBaseCounts = 0;
    g_motorNoYaw.turnYawBaseX100 = 0;
    g_motorNoYaw.turnYawDeltaX100 = 0U;
    g_motorNoYaw.turnYawFrameCount = 0U;
    g_motorNoYaw.turnCount = 0U;
    g_motorNoYaw.phase = 0U;
    g_motorNoYaw.hasTurnEncoderBase = 0U;
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
    g_motorNoYaw.rightTurnRequest = 0U;
    g_motorNoYaw.returnLineRequest = 0U;
}

void MotorNoYaw_Init(void)
{
    g_motorNoYaw.profile = MOTOR_NO_YAW_PROFILE_TASK1;
    MotorNoYaw_ResetControl();
    g_motorNoYaw.running = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_IDLE;
}

static void MotorNoYaw_StartWithProfile(MotorNoYawProfile profile)
{
    g_motorNoYaw.profile = profile;
    MotorNoYaw_ResetControl();
    MotorNoYaw_UpdateFilteredLineMask(Gray_ReadDigitalMaskFast());
    g_motorNoYaw.running = 1U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_LINE;
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
    if (g_motorNoYaw.state != MOTOR_NO_YAW_STATE_IDLE) {
        Motion_SetChassisPeriodCommand(0, 0);
    MotorEnable_SetChassis(0U);
    }
    g_motorNoYaw.running = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_IDLE;
}

void MotorNoYaw_HandleFastEvent(void)
{
    if (g_motorNoYaw.running == 0U) {
        return;
    }

    if ((g_motorNoYaw.state == MOTOR_NO_YAW_STATE_LINE) &&
        (g_motorNoYaw.rightTurnRequest != 0U)) {
        g_motorNoYaw.rightTurnRequest = 0U;
        if (MotorNoYaw_CanStartRightTurn() != 0U) {
            MotorNoYaw_ResetTurnEncoderGap();
            MotorNoYaw_StartTurnApproach();
        }
        return;
    }

    if ((g_motorNoYaw.state == MOTOR_NO_YAW_STATE_TURN_RIGHT) &&
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
    case MOTOR_NO_YAW_STATE_TURN_RIGHT:
        MotorNoYaw_TaskTurnRight();
        break;
    case MOTOR_NO_YAW_STATE_TURN_EXIT:
        MotorNoYaw_TaskTurnExit();
        break;
    case MOTOR_NO_YAW_STATE_TURN_LEFT:
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
