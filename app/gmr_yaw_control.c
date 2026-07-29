#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "gmr_yaw_control.h"

#include "FreeRTOS.h"
#include "task.h"

#include "control_config.h"
#include "encoder_motor.h"
#include "m0_attitude_link.h"

typedef struct {
    GmrYawControlSnapshot snapshot;
    uint32_t lastFrameCount;
    TickType_t lastFrameTick;
    int32_t startupYawX100;
    uint8_t holdRequested;
} GmrYawControlState;

static GmrYawControlState g_yawControl;

static uint32_t GmrYawControl_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void GmrYawControl_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static int32_t GmrYawControl_Abs32(int32_t value)
{
    return (value < 0) ? -(value + 1) + 1 : value;
}

/* UART ISR只写底层快照；本层记录首帧航向并维护数据新鲜度。 */
static void GmrYawControl_UpdateSensor(void)
{
    M0AttitudeLinkSnapshot attitude;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    uint32_t primask;

    if (M0AttitudeLink_GetSnapshot(&attitude) == 0U) {
        primask = GmrYawControl_EnterCritical();
        g_yawControl.snapshot.sensorFresh = 0U;
        g_yawControl.snapshot.sensorAgeMs = 0xFFFFFFFFUL;
        GmrYawControl_ExitCritical(primask);
        return;
    }

    primask = GmrYawControl_EnterCritical();
    if (attitude.frameCount != g_yawControl.lastFrameCount) {
        g_yawControl.lastFrameCount = attitude.frameCount;
        g_yawControl.lastFrameTick = (TickType_t)now;
        g_yawControl.snapshot.yawX100 = attitude.yawUnwrappedX100;
        g_yawControl.snapshot.yawRateX100PerSec =
            attitude.yawRateX100PerSec;
        if (g_yawControl.snapshot.referenceLocked == 0U) {
            g_yawControl.startupYawX100 = attitude.yawUnwrappedX100;
            g_yawControl.snapshot.targetYawX100 =
                g_yawControl.startupYawX100;
            g_yawControl.snapshot.referenceLocked = 1U;
        }
    }
    g_yawControl.snapshot.sensorAgeMs =
        now - (uint32_t)g_yawControl.lastFrameTick;
    g_yawControl.snapshot.sensorFresh =
        (uint8_t)((g_yawControl.snapshot.referenceLocked != 0U) &&
            (g_yawControl.snapshot.sensorAgeMs <=
                (uint32_t)GMR_M0_YAW_STALE_MS));
    GmrYawControl_ExitCritical(primask);
}

void GmrYawControl_Init(void)
{
    uint32_t primask = GmrYawControl_EnterCritical();

    g_yawControl.snapshot.active = 0U;
    g_yawControl.snapshot.sensorFresh = 0U;
    g_yawControl.snapshot.referenceLocked = 0U;
    g_yawControl.snapshot.targetReached = 0U;
    g_yawControl.snapshot.yawX100 = 0;
    g_yawControl.snapshot.targetYawX100 = 0;
    g_yawControl.snapshot.errorX100 = 0;
    g_yawControl.snapshot.yawRateX100PerSec = 0;
    g_yawControl.snapshot.sensorAgeMs = 0xFFFFFFFFUL;
    g_yawControl.snapshot.baseCommandCounts = 0;
    g_yawControl.snapshot.wheelCommandCounts = 0;
    g_yawControl.snapshot.leftTargetCounts = 0;
    g_yawControl.snapshot.rightTargetCounts = 0;
    g_yawControl.lastFrameCount = 0U;
    g_yawControl.lastFrameTick = 0U;
    g_yawControl.startupYawX100 = 0;
    g_yawControl.holdRequested = 0U;
    GmrYawControl_ExitCritical(primask);
}

void GmrYawControl_Observe(void)
{
    GmrYawControl_UpdateSensor();
}

void GmrYawControl_StartHold(void)
{
    uint32_t primask;

    GmrYawControl_UpdateSensor();
    primask = GmrYawControl_EnterCritical();
    g_yawControl.snapshot.active = 0U;
    g_yawControl.snapshot.targetReached = 0U;
    g_yawControl.snapshot.errorX100 = 0;
    g_yawControl.snapshot.baseCommandCounts =
        (int16_t)GMR_M0_YAW_BASE_SPEED_COUNTS_PER_PERIOD;
    g_yawControl.snapshot.wheelCommandCounts = 0;
    g_yawControl.snapshot.leftTargetCounts = 0;
    g_yawControl.snapshot.rightTargetCounts = 0;
    g_yawControl.holdRequested = 1U;
    if ((g_yawControl.snapshot.referenceLocked != 0U) &&
        (g_yawControl.snapshot.sensorFresh != 0U)) {
        g_yawControl.snapshot.targetYawX100 =
            g_yawControl.startupYawX100;
        g_yawControl.snapshot.errorX100 =
            g_yawControl.snapshot.yawX100 -
            g_yawControl.snapshot.targetYawX100;
        g_yawControl.snapshot.active = 1U;
        g_yawControl.holdRequested = 0U;
    }
    GmrYawControl_ExitCritical(primask);

    EncoderMotor_EnterCalibration();
}

uint8_t GmrYawControl_StartRelative(int16_t angleDegrees)
{
    uint32_t primask;

    GmrYawControl_UpdateSensor();
    primask = GmrYawControl_EnterCritical();
    if (g_yawControl.snapshot.sensorFresh == 0U) {
        GmrYawControl_ExitCritical(primask);
        return 0U;
    }
    g_yawControl.snapshot.targetYawX100 =
        g_yawControl.snapshot.yawX100 + (int32_t)angleDegrees * 100L;
    g_yawControl.snapshot.errorX100 = -(int32_t)angleDegrees * 100L;
    g_yawControl.snapshot.baseCommandCounts =
        (int16_t)GMR_M0_YAW_BASE_SPEED_COUNTS_PER_PERIOD;
    g_yawControl.snapshot.wheelCommandCounts = 0;
    g_yawControl.snapshot.leftTargetCounts = 0;
    g_yawControl.snapshot.rightTargetCounts = 0;
    g_yawControl.snapshot.targetReached = 0U;
    g_yawControl.snapshot.active = 1U;
    g_yawControl.holdRequested = 0U;
    GmrYawControl_ExitCritical(primask);

    EncoderMotor_EnterCalibration();
    return 1U;
}

void GmrYawControl_Stop(void)
{
    uint32_t primask = GmrYawControl_EnterCritical();
    uint8_t hadRequest = (uint8_t)((g_yawControl.snapshot.active != 0U) ||
        (g_yawControl.holdRequested != 0U));

    g_yawControl.snapshot.active = 0U;
    g_yawControl.snapshot.targetReached = 0U;
    g_yawControl.snapshot.targetYawX100 = g_yawControl.snapshot.yawX100;
    g_yawControl.snapshot.errorX100 = 0;
    g_yawControl.snapshot.baseCommandCounts = 0;
    g_yawControl.snapshot.wheelCommandCounts = 0;
    g_yawControl.snapshot.leftTargetCounts = 0;
    g_yawControl.snapshot.rightTargetCounts = 0;
    g_yawControl.holdRequested = 0U;
    GmrYawControl_ExitCritical(primask);
    if (hadRequest != 0U) {
        EncoderMotor_SetCalibrationTargets(0, 0);
    }
}

void GmrYawControl_RunControlPeriod(void)
{
    uint32_t primask;
    int32_t error;
    int32_t magnitude;
    int32_t speed;
    int16_t correction;
    int16_t baseCommand;
    int16_t leftTarget;
    int16_t rightTarget;
    uint8_t active;
    uint8_t fresh;

    GmrYawControl_UpdateSensor();
    primask = GmrYawControl_EnterCritical();
    if ((g_yawControl.holdRequested != 0U) &&
        (g_yawControl.snapshot.referenceLocked != 0U) &&
        (g_yawControl.snapshot.sensorFresh != 0U)) {
        g_yawControl.snapshot.targetYawX100 =
            g_yawControl.startupYawX100;
        g_yawControl.snapshot.active = 1U;
        g_yawControl.holdRequested = 0U;
    }
    active = g_yawControl.snapshot.active;
    fresh = g_yawControl.snapshot.sensorFresh;
    baseCommand = g_yawControl.snapshot.baseCommandCounts;
    error = g_yawControl.snapshot.yawX100 -
        g_yawControl.snapshot.targetYawX100;
    g_yawControl.snapshot.errorX100 = error;
    GmrYawControl_ExitCritical(primask);
    if (active == 0U) {
        return;
    }
    if (fresh == 0U) {
        GmrYawControl_Stop();
        return;
    }

    magnitude = GmrYawControl_Abs32(error);
    if (magnitude <= (int32_t)GMR_M0_YAW_DEADBAND_X100) {
        correction = 0;
    } else {
        speed = (magnitude * (int32_t)GMR_M0_YAW_KP_Q1024) /
            (int32_t)CHASSIS_Q1024_SCALE;
        if (speed < (int32_t)GMR_M0_YAW_MIN_CORRECTION_COUNTS_PER_PERIOD) {
            speed = (int32_t)GMR_M0_YAW_MIN_CORRECTION_COUNTS_PER_PERIOD;
        }
        if (speed > (int32_t)GMR_M0_YAW_MAX_CORRECTION_COUNTS_PER_PERIOD) {
            speed = (int32_t)GMR_M0_YAW_MAX_CORRECTION_COUNTS_PER_PERIOD;
        }
        correction = (int16_t)((error < 0) ? -speed : speed);
        correction = (int16_t)(correction *
            (int16_t)GMR_M0_YAW_CHASSIS_DIRECTION_SIGN);
    }
    leftTarget = (int16_t)(baseCommand - correction);
    rightTarget = (int16_t)(baseCommand + correction);

    primask = GmrYawControl_EnterCritical();
    g_yawControl.snapshot.targetReached = (uint8_t)(correction == 0);
    g_yawControl.snapshot.wheelCommandCounts = correction;
    g_yawControl.snapshot.leftTargetCounts = leftTarget;
    g_yawControl.snapshot.rightTargetCounts = rightTarget;
    GmrYawControl_ExitCritical(primask);
    EncoderMotor_SetCalibrationTargets(leftTarget, rightTarget);
}

void GmrYawControl_GetSnapshot(GmrYawControlSnapshot *snapshot)
{
    uint32_t primask;

    if (snapshot == 0) {
        return;
    }
    primask = GmrYawControl_EnterCritical();
    *snapshot = g_yawControl.snapshot;
    GmrYawControl_ExitCritical(primask);
}

#endif
