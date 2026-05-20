#include "tracking_exception.h"

#include "board_config.h"

static TrackingExceptionState g_trackingExceptionState;
static uint16_t g_lostTicks;
static uint16_t g_adcFaultTicks;
static int8_t g_lastLineSide;
static int16_t g_lastLeftDuty;
static int16_t g_lastRightDuty;
static uint8_t g_hasLastCommand;

/*
 * 作用：限制电机命令范围。
 * 使用场景：异常处理器自己生成保持/搜线速度时。
 */
static int16_t TrackingException_ClampDuty(int32_t duty)
{
    if (duty > (int32_t)CAR_MOTOR_PWM_MAX_COUNTS) {
        return (int16_t)CAR_MOTOR_PWM_MAX_COUNTS;
    }
    if (duty < -(int32_t)CAR_MOTOR_PWM_MAX_COUNTS) {
        return (int16_t)(-(int32_t)CAR_MOTOR_PWM_MAX_COUNTS);
    }
    return (int16_t)duty;
}

/*
 * 作用：统计 7 路灰度黑白结果里有多少路压线。
 * 使用场景：识别丢线和宽线/路口。
 */
static uint8_t TrackingException_CountActiveSensors(uint8_t digitalMask)
{
    uint8_t count = 0U;

    while (digitalMask != 0U) {
        if ((digitalMask & 0x01U) != 0U) {
            ++count;
        }
        digitalMask >>= 1U;
    }

    return count;
}

/*
 * 作用：保存最近一次有效电机命令。
 * 使用场景：短暂丢线时先保持上一拍输出，避免遇到小断点就急停。
 */
static void TrackingException_SaveCommand(int16_t leftDuty, int16_t rightDuty)
{
    g_lastLeftDuty = leftDuty;
    g_lastRightDuty = rightDuty;
    g_hasLastCommand = 1U;
}

/*
 * 作用：根据最后一次偏差决定向哪边搜线。
 * 使用场景：丢线进入搜线阶段时。
 * 说明：偏差为负表示线在左边，搜线也优先往左打方向。
 */
static uint8_t TrackingException_ShouldSearchLeft(void)
{
    if (g_lastLineSide < 0) {
        return 1U;
    }
    if (g_lastLineSide > 0) {
        return 0U;
    }

    return CAR_TRACK_SEARCH_DEFAULT_LEFT ? 1U : 0U;
}

/*
 * 作用：生成正常循迹时的左右轮命令。
 * 使用场景：灰度数据有效且能看到线时。
 */
static void TrackingException_BuildNormalCommand(int16_t lineError,
    uint16_t baseDuty, uint16_t turnLimit, int16_t *leftDuty,
    int16_t *rightDuty)
{
    int16_t correction =
        (int16_t)((lineError * CAR_TRACK_TURN_GAIN) / GRAY_LINE_ERROR_SCALE);

    if (correction > (int16_t)turnLimit) {
        correction = (int16_t)turnLimit;
    } else if (correction < -(int16_t)turnLimit) {
        correction = -(int16_t)turnLimit;
    }

    *leftDuty = TrackingException_ClampDuty(
        (int32_t)baseDuty + (int32_t)correction);
    *rightDuty = TrackingException_ClampDuty(
        (int32_t)baseDuty - (int32_t)correction);
}

/*
 * 作用：生成丢线后的温和搜线命令。
 * 使用场景：短暂保持仍没找回线时。
 * 说明：只做前进差速搜线，不直接原地反转，避免动作过猛。
 */
static void TrackingException_BuildSearchCommand(uint8_t searchLeft,
    int16_t *leftDuty, int16_t *rightDuty)
{
    int32_t slowDuty = (int32_t)CAR_TRACK_LOST_SEARCH_BASE_DUTY -
        (int32_t)CAR_TRACK_LOST_SEARCH_DELTA_DUTY;
    int32_t fastDuty = (int32_t)CAR_TRACK_LOST_SEARCH_BASE_DUTY +
        (int32_t)CAR_TRACK_LOST_SEARCH_DELTA_DUTY;

    if (slowDuty < 0) {
        slowDuty = 0;
    }
    if (fastDuty > (int32_t)CAR_MOTOR_PWM_MAX_COUNTS) {
        fastDuty = (int32_t)CAR_MOTOR_PWM_MAX_COUNTS;
    }

    if (searchLeft) {
        *leftDuty = (int16_t)slowDuty;
        *rightDuty = (int16_t)fastDuty;
        g_trackingExceptionState =
            TRACKING_EXCEPTION_STATE_LOST_SEARCH_LEFT;
    } else {
        *leftDuty = (int16_t)fastDuty;
        *rightDuty = (int16_t)slowDuty;
        g_trackingExceptionState =
            TRACKING_EXCEPTION_STATE_LOST_SEARCH_RIGHT;
    }
}

/*
 * 作用：处理 ADC 采样失败。
 * 使用场景：Gray_Update 返回失败时。
 */
static TrackingExceptionAction TrackingException_HandleAdcFault(
    int16_t *leftDuty, int16_t *rightDuty)
{
    if (g_adcFaultTicks < 0xFFFFU) {
        ++g_adcFaultTicks;
    }

    g_trackingExceptionState = TRACKING_EXCEPTION_STATE_ADC_FAULT;
    if ((g_adcFaultTicks < CAR_TRACK_ADC_FAULT_STOP_TICKS) &&
        g_hasLastCommand) {
        *leftDuty = g_lastLeftDuty;
        *rightDuty = g_lastRightDuty;
        return TRACKING_EXCEPTION_ACTION_RUN;
    }

    *leftDuty = 0;
    *rightDuty = 0;
    return TRACKING_EXCEPTION_ACTION_STOP;
}

/*
 * 作用：处理灰度传感器看不到线的情况。
 * 使用场景：digitalMask 为 0 时。
 */
static TrackingExceptionAction TrackingException_HandleLostLine(
    int16_t *leftDuty, int16_t *rightDuty)
{
    uint16_t searchStopTicks =
        (uint16_t)(CAR_TRACK_LOST_HOLD_TICKS +
            CAR_TRACK_LOST_SEARCH_TICKS);

    if (g_lostTicks < 0xFFFFU) {
        ++g_lostTicks;
    }

    if (g_lostTicks <= CAR_TRACK_LOST_HOLD_TICKS) {
        g_trackingExceptionState = TRACKING_EXCEPTION_STATE_LOST_HOLD;
        if (g_hasLastCommand) {
            *leftDuty = g_lastLeftDuty;
            *rightDuty = g_lastRightDuty;
        } else {
            *leftDuty = 0;
            *rightDuty = 0;
        }
        return TRACKING_EXCEPTION_ACTION_RUN;
    }

    if (g_lostTicks <= searchStopTicks) {
        TrackingException_BuildSearchCommand(
            TrackingException_ShouldSearchLeft(), leftDuty, rightDuty);
        TrackingException_SaveCommand(*leftDuty, *rightDuty);
        return TRACKING_EXCEPTION_ACTION_RUN;
    }

#if CAR_TRACK_LOST_STOP
    g_trackingExceptionState = TRACKING_EXCEPTION_STATE_LOST_STOP;
    *leftDuty = 0;
    *rightDuty = 0;
    return TRACKING_EXCEPTION_ACTION_STOP;
#else
    TrackingException_BuildSearchCommand(
        TrackingException_ShouldSearchLeft(), leftDuty, rightDuty);
    TrackingException_SaveCommand(*leftDuty, *rightDuty);
    return TRACKING_EXCEPTION_ACTION_RUN;
#endif
}

void TrackingException_Init(void)
{
    TrackingException_Reset();
}

void TrackingException_Reset(void)
{
    g_trackingExceptionState = TRACKING_EXCEPTION_STATE_NORMAL;
    g_lostTicks = 0U;
    g_adcFaultTicks = 0U;
    g_lastLineSide = 0;
    g_lastLeftDuty = 0;
    g_lastRightDuty = 0;
    g_hasLastCommand = 0U;
}

TrackingExceptionAction TrackingException_Update(uint8_t sampleOk,
    uint8_t digitalMask, int16_t lineError, uint16_t baseDuty,
    uint16_t turnLimit, int16_t *leftDuty, int16_t *rightDuty)
{
    uint8_t activeCount;

    if ((leftDuty == 0) || (rightDuty == 0)) {
        return TRACKING_EXCEPTION_ACTION_STOP;
    }

    if (!sampleOk) {
        return TrackingException_HandleAdcFault(leftDuty, rightDuty);
    }

    g_adcFaultTicks = 0U;
    activeCount = TrackingException_CountActiveSensors(digitalMask);

    if (activeCount == 0U) {
        return TrackingException_HandleLostLine(leftDuty, rightDuty);
    }

    g_lostTicks = 0U;
    if (lineError < 0) {
        g_lastLineSide = -1;
    } else if (lineError > 0) {
        g_lastLineSide = 1;
    }

    TrackingException_BuildNormalCommand(
        lineError, baseDuty, turnLimit, leftDuty, rightDuty);
    TrackingException_SaveCommand(*leftDuty, *rightDuty);

    if (activeCount >= CAR_TRACK_WIDE_LINE_ACTIVE_COUNT) {
        g_trackingExceptionState = TRACKING_EXCEPTION_STATE_WIDE_LINE;
    } else {
        g_trackingExceptionState = TRACKING_EXCEPTION_STATE_NORMAL;
    }

    return TRACKING_EXCEPTION_ACTION_RUN;
}

TrackingExceptionState TrackingException_GetState(void)
{
    return g_trackingExceptionState;
}

const char *TrackingException_GetStateName(TrackingExceptionState state)
{
    switch (state) {
    case TRACKING_EXCEPTION_STATE_NORMAL:
        return "normal";
    case TRACKING_EXCEPTION_STATE_WIDE_LINE:
        return "wide_line";
    case TRACKING_EXCEPTION_STATE_LOST_HOLD:
        return "lost_hold";
    case TRACKING_EXCEPTION_STATE_LOST_SEARCH_LEFT:
        return "search_left";
    case TRACKING_EXCEPTION_STATE_LOST_SEARCH_RIGHT:
        return "search_right";
    case TRACKING_EXCEPTION_STATE_ADC_FAULT:
        return "adc_fault";
    case TRACKING_EXCEPTION_STATE_LOST_STOP:
        return "lost_stop";
    default:
        return "unknown";
    }
}

uint16_t TrackingException_GetLostTicks(void)
{
    return g_lostTicks;
}

uint16_t TrackingException_GetAdcFaultTicks(void)
{
    return g_adcFaultTicks;
}
