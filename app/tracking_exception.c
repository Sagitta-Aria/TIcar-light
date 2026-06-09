#include "tracking_exception.h"

#include "board_config.h"
#include "log_uart.h"

static TrackingExceptionState g_trackingExceptionState;
static uint16_t g_lostTicks;
static uint16_t g_sensorFaultTicks;
static int8_t g_lastLineSide;
static int16_t g_lastLeftSpeedSps;
static int16_t g_lastRightSpeedSps;
static uint8_t g_hasLastSpeed;

/*
 * 作用：把循迹异常状态转成日志字符串。
 * 使用场景：状态变化时串口打印，后续也可用于 OLED 调试页。
 */
static const char *TrackingException_StateText(TrackingExceptionState state)
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
    case TRACKING_EXCEPTION_STATE_SENSOR_FAULT:
        return "sensor_fault";
    case TRACKING_EXCEPTION_STATE_LOST_STOP:
        return "lost_stop";
    default:
        return "unknown";
    }
}

/*
 * 作用：切换异常状态并在变化时打印日志。
 * 使用场景：丢线、宽线、灰度故障等状态迁移。
 */
static void TrackingException_SetState(TrackingExceptionState state)
{
    if (g_trackingExceptionState != state) {
        LOG_RAW("tracking exception: ");
        LOG_LINE(TrackingException_StateText(state));
    }
    g_trackingExceptionState = state;
}

/*
 * 作用：限制电机 SPS 范围。
 * 使用场景：异常处理器自己生成保持/搜线速度时。
 */
static int16_t TrackingException_ClampSpeedSps(int32_t speedSps)
{
    if (speedSps > (int32_t)CAR_STEPPER_SPEED_MAX_SPS) {
        return (int16_t)CAR_STEPPER_SPEED_MAX_SPS;
    }
    if (speedSps < -(int32_t)CAR_STEPPER_SPEED_MAX_SPS) {
        return (int16_t)(-(int32_t)CAR_STEPPER_SPEED_MAX_SPS);
    }
    return (int16_t)speedSps;
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
 * 作用：保存最近一次有效电机 SPS。
 * 使用场景：短暂丢线时先保持上一拍输出，避免遇到小断点就急停。
 */
static void TrackingException_SaveSpeed(int16_t leftSpeedSps,
    int16_t rightSpeedSps)
{
    g_lastLeftSpeedSps = leftSpeedSps;
    g_lastRightSpeedSps = rightSpeedSps;
    g_hasLastSpeed = 1U;
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
 * 作用：生成正常循迹时的左右轮 SPS。
 * 使用场景：灰度数据有效且能看到线时。
 */
static void TrackingException_BuildNormalSpeed(int16_t lineError,
    uint16_t baseSpeedSps, uint16_t turnLimit, int16_t *leftSpeedSps,
    int16_t *rightSpeedSps)
{
    int16_t correction =
        (int16_t)((lineError * CAR_TRACK_TURN_GAIN) / GRAY_LINE_ERROR_SCALE);

    if (correction > (int16_t)turnLimit) {
        correction = (int16_t)turnLimit;
    } else if (correction < -(int16_t)turnLimit) {
        correction = -(int16_t)turnLimit;
    }

    *leftSpeedSps = TrackingException_ClampSpeedSps(
        (int32_t)baseSpeedSps + (int32_t)correction);
    *rightSpeedSps = TrackingException_ClampSpeedSps(
        (int32_t)baseSpeedSps - (int32_t)correction);
}

/*
 * 作用：生成丢线后的温和搜线 SPS。
 * 使用场景：短暂保持仍没找回线时。
 * 说明：只做前进差速搜线，不直接原地反转，避免动作过猛。
 */
static void TrackingException_BuildSearchSpeed(uint8_t searchLeft,
    int16_t *leftSpeedSps, int16_t *rightSpeedSps)
{
    int32_t slowSpeedSps = (int32_t)CAR_TRACK_LOST_SEARCH_BASE_SPEED_SPS -
        (int32_t)CAR_TRACK_LOST_SEARCH_DELTA_SPS;
    int32_t fastSpeedSps = (int32_t)CAR_TRACK_LOST_SEARCH_BASE_SPEED_SPS +
        (int32_t)CAR_TRACK_LOST_SEARCH_DELTA_SPS;

    if (slowSpeedSps < 0) {
        slowSpeedSps = 0;
    }
    if (fastSpeedSps > (int32_t)CAR_STEPPER_SPEED_MAX_SPS) {
        fastSpeedSps = (int32_t)CAR_STEPPER_SPEED_MAX_SPS;
    }

    if (searchLeft) {
        *leftSpeedSps = (int16_t)slowSpeedSps;
        *rightSpeedSps = (int16_t)fastSpeedSps;
        TrackingException_SetState(TRACKING_EXCEPTION_STATE_LOST_SEARCH_LEFT);
    } else {
        *leftSpeedSps = (int16_t)fastSpeedSps;
        *rightSpeedSps = (int16_t)slowSpeedSps;
        TrackingException_SetState(TRACKING_EXCEPTION_STATE_LOST_SEARCH_RIGHT);
    }
}

/*
 * 作用：处理灰度采样失败。
 * 使用场景：Gray_Update 返回失败时。
 */
static TrackingExceptionAction TrackingException_HandleSensorFault(
    int16_t *leftSpeedSps, int16_t *rightSpeedSps)
{
    if (g_sensorFaultTicks < 0xFFFFU) {
        ++g_sensorFaultTicks;
    }

    TrackingException_SetState(TRACKING_EXCEPTION_STATE_SENSOR_FAULT);
    if ((g_sensorFaultTicks < CAR_TRACK_SENSOR_FAULT_STOP_TICKS) &&
        g_hasLastSpeed) {
        *leftSpeedSps = g_lastLeftSpeedSps;
        *rightSpeedSps = g_lastRightSpeedSps;
        return TRACKING_EXCEPTION_ACTION_RUN;
    }

    *leftSpeedSps = 0;
    *rightSpeedSps = 0;
    return TRACKING_EXCEPTION_ACTION_STOP;
}

/*
 * 作用：处理灰度传感器看不到线的情况。
 * 使用场景：digitalMask 为 0 时。
 */
static TrackingExceptionAction TrackingException_HandleLostLine(
    int16_t *leftSpeedSps, int16_t *rightSpeedSps)
{
    uint16_t searchStopTicks =
        (uint16_t)(CAR_TRACK_LOST_HOLD_TICKS +
            CAR_TRACK_LOST_SEARCH_TICKS);

    if (g_lostTicks < 0xFFFFU) {
        ++g_lostTicks;
    }

    if (g_lostTicks <= CAR_TRACK_LOST_HOLD_TICKS) {
        TrackingException_SetState(TRACKING_EXCEPTION_STATE_LOST_HOLD);
        if (g_hasLastSpeed) {
            *leftSpeedSps = g_lastLeftSpeedSps;
            *rightSpeedSps = g_lastRightSpeedSps;
        } else {
            *leftSpeedSps = 0;
            *rightSpeedSps = 0;
        }
        return TRACKING_EXCEPTION_ACTION_RUN;
    }

    if (g_lostTicks <= searchStopTicks) {
        TrackingException_BuildSearchSpeed(
            TrackingException_ShouldSearchLeft(), leftSpeedSps, rightSpeedSps);
        TrackingException_SaveSpeed(*leftSpeedSps, *rightSpeedSps);
        return TRACKING_EXCEPTION_ACTION_RUN;
    }

#if CAR_TRACK_LOST_STOP
    TrackingException_SetState(TRACKING_EXCEPTION_STATE_LOST_STOP);
    *leftSpeedSps = 0;
    *rightSpeedSps = 0;
    return TRACKING_EXCEPTION_ACTION_STOP;
#else
    TrackingException_BuildSearchSpeed(
        TrackingException_ShouldSearchLeft(), leftSpeedSps, rightSpeedSps);
    TrackingException_SaveSpeed(*leftSpeedSps, *rightSpeedSps);
    return TRACKING_EXCEPTION_ACTION_RUN;
#endif
}

/*
 * 作用：初始化循迹异常处理器。
 * 使用场景：Tracking_Init 调用一次。
 */
void TrackingException_Init(void)
{
    TrackingException_Reset();
}

/*
 * 作用：清空丢线、灰度故障和上一拍命令缓存。
 * 使用场景：每次重新启用循迹或退出循迹时。
 */
void TrackingException_Reset(void)
{
    g_trackingExceptionState = TRACKING_EXCEPTION_STATE_NORMAL;
    g_lostTicks = 0U;
    g_sensorFaultTicks = 0U;
    g_lastLineSide = 0;
    g_lastLeftSpeedSps = 0;
    g_lastRightSpeedSps = 0;
    g_hasLastSpeed = 0U;
}

/*
 * 作用：根据灰度采样结果生成安全的左右轮 SPS。
 * 使用场景：Tracking_Task 每轮调用。
 * 说明：输入命令指针不能为空；返回 STOP 时调用者应立即停车。
 */
TrackingExceptionAction TrackingException_Update(uint8_t sampleOk,
    uint8_t digitalMask, int16_t lineError, uint16_t baseSpeedSps,
    uint16_t turnLimit, int16_t *leftSpeedSps, int16_t *rightSpeedSps)
{
    uint8_t activeCount;

    if ((leftSpeedSps == 0) || (rightSpeedSps == 0)) {
        return TRACKING_EXCEPTION_ACTION_STOP;
    }

    if (!sampleOk) {
        return TrackingException_HandleSensorFault(leftSpeedSps, rightSpeedSps);
    }

    g_sensorFaultTicks = 0U;
    activeCount = TrackingException_CountActiveSensors(digitalMask);

    if (activeCount == 0U) {
        return TrackingException_HandleLostLine(leftSpeedSps, rightSpeedSps);
    }

    g_lostTicks = 0U;
    if (lineError < 0) {
        g_lastLineSide = -1;
    } else if (lineError > 0) {
        g_lastLineSide = 1;
    }

    TrackingException_BuildNormalSpeed(
        lineError, baseSpeedSps, turnLimit, leftSpeedSps, rightSpeedSps);
    TrackingException_SaveSpeed(*leftSpeedSps, *rightSpeedSps);

    if (activeCount >= CAR_TRACK_WIDE_LINE_ACTIVE_COUNT) {
        TrackingException_SetState(TRACKING_EXCEPTION_STATE_WIDE_LINE);
    } else {
        TrackingException_SetState(TRACKING_EXCEPTION_STATE_NORMAL);
    }

    return TRACKING_EXCEPTION_ACTION_RUN;
}

/* 作用：读取当前循迹异常状态。 */
TrackingExceptionState TrackingException_GetState(void)
{
    return g_trackingExceptionState;
}

/* 作用：把指定异常状态转成字符串。 */
const char *TrackingException_GetStateName(TrackingExceptionState state)
{
    return TrackingException_StateText(state);
}

/* 作用：读取连续丢线计数。 */
uint16_t TrackingException_GetLostTicks(void)
{
    return g_lostTicks;
}

/* 作用：读取连续灰度采样失败计数。 */
uint16_t TrackingException_GetSensorFaultTicks(void)
{
    return g_sensorFaultTicks;
}
