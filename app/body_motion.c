#include "body_motion.h"

#include "FreeRTOS.h"
#include "task.h"

#include "control_config.h"
#include "jy61p.h"

#define BODY_MOTION_Q1024_SCALE          (1024L)
#define BODY_MOTION_FULL_TURN_X100       (36000L)
#define BODY_MOTION_HALF_TURN_X100       (18000L)
#define BODY_MOTION_MAX_DT_MS            (100U)

typedef struct {
    BodyMotionConfig config;
    BodyMotionSnapshot snapshot;
    uint32_t lastAngleFrameCount;
    uint32_t lastGyroFrameCount;
    uint32_t lastIntegratedGyroTick;
    int16_t lastYawRawX100;
    int64_t biasSumX100PerSec;
    uint8_t hasYaw;
    uint8_t hasGyroTick;
    uint8_t initialized;
} BodyMotionControl;

static BodyMotionControl g_bodyMotion;

static uint32_t BodyMotion_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void BodyMotion_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static int32_t BodyMotion_ApplyQ1024(int32_t value, uint16_t gainQ1024)
{
    int64_t product = (int64_t)value * gainQ1024;

    if (product >= 0) {
        return (product + BODY_MOTION_Q1024_SCALE / 2L) /
            BODY_MOTION_Q1024_SCALE;
    }
    return -((-product + BODY_MOTION_Q1024_SCALE / 2L) /
        BODY_MOTION_Q1024_SCALE);
}

static int32_t BodyMotion_IntegrateRate(int32_t rateX100PerSec,
    uint32_t dtMs)
{
    int32_t product = rateX100PerSec * (int32_t)dtMs;

    if (product >= 0) {
        return (product + 500L) / 1000L;
    }
    return -((-product + 500L) / 1000L);
}

static uint32_t BodyMotion_GetAgeMs(uint32_t now, uint32_t frameTick,
    uint32_t frameCount)
{
    return (frameCount == 0U) ? 0xFFFFFFFFUL : now - frameTick;
}

static void BodyMotion_UpdateYaw(const JY61P_Attitude *attitude)
{
    int32_t delta;

    if (attitude->angleFrameCount == g_bodyMotion.lastAngleFrameCount) {
        return;
    }
    g_bodyMotion.lastAngleFrameCount = attitude->angleFrameCount;
    g_bodyMotion.snapshot.yawRawX100 = attitude->yawX100;
    if (g_bodyMotion.hasYaw == 0U) {
        g_bodyMotion.hasYaw = 1U;
        g_bodyMotion.lastYawRawX100 = attitude->yawX100;
        g_bodyMotion.snapshot.yawUnwrappedX100 = attitude->yawX100;
        g_bodyMotion.snapshot.yawEstimateX100 = attitude->yawX100;
        return;
    }

    delta = (int32_t)attitude->yawX100 -
        (int32_t)g_bodyMotion.lastYawRawX100;
    if (delta > BODY_MOTION_HALF_TURN_X100) {
        delta -= BODY_MOTION_FULL_TURN_X100;
    } else if (delta < -BODY_MOTION_HALF_TURN_X100) {
        delta += BODY_MOTION_FULL_TURN_X100;
    }
    g_bodyMotion.lastYawRawX100 = attitude->yawX100;
    g_bodyMotion.snapshot.yawUnwrappedX100 += delta;
}

static void BodyMotion_UpdateGyro(const JY61P_Attitude *attitude)
{
    int32_t corrected;
    int32_t difference;

    if (attitude->gyroFrameCount == g_bodyMotion.lastGyroFrameCount) {
        return;
    }
    g_bodyMotion.lastGyroFrameCount = attitude->gyroFrameCount;
    g_bodyMotion.snapshot.yawRateRawX100PerSec =
        attitude->yawRateX100PerSec;

    if (g_bodyMotion.snapshot.state == BODY_MOTION_CALIBRATING) {
        g_bodyMotion.biasSumX100PerSec += attitude->yawRateX100PerSec;
        if (g_bodyMotion.snapshot.calibrationCount < 0xFFFFU) {
            ++g_bodyMotion.snapshot.calibrationCount;
        }
        if (g_bodyMotion.snapshot.calibrationCount >=
            g_bodyMotion.config.calibrationSamples) {
            g_bodyMotion.snapshot.gyroBiasX100PerSec =
                (int32_t)(g_bodyMotion.biasSumX100PerSec /
                    g_bodyMotion.snapshot.calibrationCount);
            g_bodyMotion.snapshot.yawRateFilteredX100PerSec = 0;
            g_bodyMotion.snapshot.state = (g_bodyMotion.hasYaw != 0U) ?
                BODY_MOTION_READY : BODY_MOTION_WAITING;
        }
        return;
    }

    corrected = attitude->yawRateX100PerSec -
        g_bodyMotion.snapshot.gyroBiasX100PerSec;
    difference = corrected -
        g_bodyMotion.snapshot.yawRateFilteredX100PerSec;
    g_bodyMotion.snapshot.yawRateFilteredX100PerSec +=
        BodyMotion_ApplyQ1024(difference,
            g_bodyMotion.config.gyroAlphaQ1024);
}

void BodyMotion_Init(void)
{
    g_bodyMotion.config.gyroAlphaQ1024 =
        (uint16_t)BODY_MOTION_GYRO_ALPHA_Q1024;
    g_bodyMotion.config.yawBetaQ1024 =
        (uint16_t)BODY_MOTION_YAW_BETA_Q1024;
    g_bodyMotion.config.predictionMs =
        (uint16_t)BODY_MOTION_PREDICTION_MS;
    g_bodyMotion.config.staleMs =
        (uint16_t)BODY_MOTION_SENSOR_STALE_MS;
    g_bodyMotion.config.calibrationSamples =
        (uint16_t)BODY_MOTION_CALIBRATION_SAMPLES;
    g_bodyMotion.initialized = 1U;
    g_bodyMotion.hasYaw = 0U;
    g_bodyMotion.lastAngleFrameCount = 0U;
    g_bodyMotion.lastGyroFrameCount = 0U;
    g_bodyMotion.lastIntegratedGyroTick = 0U;
    g_bodyMotion.hasGyroTick = 0U;
    g_bodyMotion.snapshot.angleFrameCount = 0U;
    g_bodyMotion.snapshot.gyroFrameCount = 0U;
    g_bodyMotion.snapshot.yawRawX100 = 0;
    g_bodyMotion.snapshot.yawUnwrappedX100 = 0;
    g_bodyMotion.snapshot.yawEstimateX100 = 0;
    g_bodyMotion.snapshot.yawControlX100 = 0;
    g_bodyMotion.snapshot.yawRateRawX100PerSec = 0;
    g_bodyMotion.snapshot.yawRateFilteredX100PerSec = 0;
    g_bodyMotion.snapshot.gyroBiasX100PerSec = 0;
    g_bodyMotion.snapshot.angleAgeMs = 0xFFFFFFFFUL;
    g_bodyMotion.snapshot.gyroAgeMs = 0xFFFFFFFFUL;
    BodyMotion_StartCalibration();
}

void BodyMotion_StartCalibration(void)
{
    uint32_t primask;

    if (g_bodyMotion.initialized == 0U) {
        return;
    }
    primask = BodyMotion_EnterCritical();
    g_bodyMotion.biasSumX100PerSec = 0;
    g_bodyMotion.snapshot.calibrationCount = 0U;
    g_bodyMotion.snapshot.calibrationTarget =
        g_bodyMotion.config.calibrationSamples;
    g_bodyMotion.snapshot.gyroBiasX100PerSec = 0;
    g_bodyMotion.snapshot.yawRateFilteredX100PerSec = 0;
    g_bodyMotion.hasGyroTick = 0U;
    g_bodyMotion.snapshot.state = BODY_MOTION_CALIBRATING;
    BodyMotion_ExitCritical(primask);
}

void BodyMotion_Task(void)
{
    JY61P_Attitude attitude;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    uint32_t dtMs = 0U;
    uint32_t predictionAgeMs;
    uint8_t newAngle;
    uint8_t newGyro;

    if (g_bodyMotion.initialized == 0U) {
        return;
    }
    if (JY61P_GetAttitude(&attitude) == 0U) {
        if (g_bodyMotion.snapshot.state != BODY_MOTION_CALIBRATING) {
            g_bodyMotion.snapshot.state = BODY_MOTION_WAITING;
        }
        return;
    }

    newAngle = (uint8_t)(attitude.angleFrameCount !=
        g_bodyMotion.lastAngleFrameCount);
    newGyro = (uint8_t)(attitude.gyroFrameCount !=
        g_bodyMotion.lastGyroFrameCount);
    BodyMotion_UpdateYaw(&attitude);
    BodyMotion_UpdateGyro(&attitude);
    g_bodyMotion.snapshot.angleFrameCount = attitude.angleFrameCount;
    g_bodyMotion.snapshot.gyroFrameCount = attitude.gyroFrameCount;
    g_bodyMotion.snapshot.angleAgeMs = BodyMotion_GetAgeMs(now,
        attitude.angleFrameTick, attitude.angleFrameCount);
    g_bodyMotion.snapshot.gyroAgeMs = BodyMotion_GetAgeMs(now,
        attitude.gyroFrameTick, attitude.gyroFrameCount);

    if (g_bodyMotion.snapshot.state == BODY_MOTION_CALIBRATING) {
        return;
    }
    if ((g_bodyMotion.snapshot.angleAgeMs > g_bodyMotion.config.staleMs) ||
        (g_bodyMotion.snapshot.gyroAgeMs > g_bodyMotion.config.staleMs)) {
        g_bodyMotion.snapshot.state = BODY_MOTION_STALE;
        g_bodyMotion.snapshot.yawRateFilteredX100PerSec = 0;
        return;
    }
    if (g_bodyMotion.hasYaw == 0U) {
        g_bodyMotion.snapshot.state = BODY_MOTION_WAITING;
        return;
    }

    if (newGyro != 0U) {
        if (g_bodyMotion.hasGyroTick != 0U) {
            dtMs = attitude.gyroFrameTick -
                g_bodyMotion.lastIntegratedGyroTick;
            if (dtMs > BODY_MOTION_MAX_DT_MS) {
                dtMs = BODY_MOTION_MAX_DT_MS;
            }
            g_bodyMotion.snapshot.yawEstimateX100 +=
                BodyMotion_IntegrateRate(
                    g_bodyMotion.snapshot.yawRateFilteredX100PerSec, dtMs);
        }
        g_bodyMotion.lastIntegratedGyroTick = attitude.gyroFrameTick;
        g_bodyMotion.hasGyroTick = 1U;
    }
    if (newAngle != 0U) {
        g_bodyMotion.snapshot.yawEstimateX100 += BodyMotion_ApplyQ1024(
            g_bodyMotion.snapshot.yawUnwrappedX100 -
                g_bodyMotion.snapshot.yawEstimateX100,
            g_bodyMotion.config.yawBetaQ1024);
    }
    predictionAgeMs = (g_bodyMotion.hasGyroTick != 0U) ?
        now - g_bodyMotion.lastIntegratedGyroTick : 0U;
    if (predictionAgeMs > g_bodyMotion.config.staleMs) {
        predictionAgeMs = g_bodyMotion.config.staleMs;
    }
    g_bodyMotion.snapshot.yawControlX100 =
        g_bodyMotion.snapshot.yawEstimateX100 + BodyMotion_IntegrateRate(
            g_bodyMotion.snapshot.yawRateFilteredX100PerSec,
            predictionAgeMs + g_bodyMotion.config.predictionMs);
    g_bodyMotion.snapshot.state = BODY_MOTION_READY;
}

void BodyMotion_GetSnapshot(BodyMotionSnapshot *snapshot)
{
    uint32_t primask;

    if (snapshot == 0) {
        return;
    }
    primask = BodyMotion_EnterCritical();
    *snapshot = g_bodyMotion.snapshot;
    BodyMotion_ExitCritical(primask);
}

void BodyMotion_GetConfig(BodyMotionConfig *config)
{
    uint32_t primask;

    if (config == 0) {
        return;
    }
    primask = BodyMotion_EnterCritical();
    *config = g_bodyMotion.config;
    BodyMotion_ExitCritical(primask);
}

uint8_t BodyMotion_SetConfig(const BodyMotionConfig *config)
{
    uint32_t primask;

    if ((config == 0) || (config->gyroAlphaQ1024 == 0U) ||
        (config->gyroAlphaQ1024 > 1024U) ||
        (config->yawBetaQ1024 == 0U) ||
        (config->yawBetaQ1024 > 1024U) ||
        (config->predictionMs > 100U) || (config->staleMs < 20U) ||
        (config->calibrationSamples < 10U)) {
        return 0U;
    }
    primask = BodyMotion_EnterCritical();
    g_bodyMotion.config = *config;
    g_bodyMotion.snapshot.calibrationTarget = config->calibrationSamples;
    BodyMotion_ExitCritical(primask);
    return 1U;
}
