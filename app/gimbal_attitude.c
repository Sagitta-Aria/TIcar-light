#include "gimbal_attitude.h"

#include "FreeRTOS.h"
#include "task.h"

#include "board_config.h"
#include "control_config.h"
#include "motor.h"

#define GIMBAL_ATTITUDE_Q1024_SCALE       (1024L)
#define GIMBAL_ATTITUDE_TURN_X100         (36000L)

typedef struct {
    GimbalAttitudeSnapshot snapshot;
    int32_t baseStep;
    int32_t baseYawX100;
} GimbalAttitudeControl;

static GimbalAttitudeControl g_gimbalAttitude;

static int32_t GimbalAttitude_ClampInt32(int64_t value)
{
    if (value > 0x7FFFFFFFLL) {
        return 0x7FFFFFFF;
    }
    if (value < -0x7FFFFFFFLL - 1LL) {
        return (int32_t)0x80000000UL;
    }
    return (int32_t)value;
}

static int32_t GimbalAttitude_DivideRounded(int64_t numerator,
    int32_t denominator)
{
    if (numerator >= 0) {
        return GimbalAttitude_ClampInt32(
            (numerator + denominator / 2) / denominator);
    }
    return GimbalAttitude_ClampInt32(
        -((-numerator + denominator / 2) / denominator));
}

static int16_t GimbalAttitude_ClampSpeed(int32_t speed)
{
    int32_t limit = g_gimbalAttitude.snapshot.config.maxSpeedSps;

    if (speed > limit) {
        return (int16_t)limit;
    }
    if (speed < -limit) {
        return (int16_t)-limit;
    }
    return (int16_t)speed;
}

static void GimbalAttitude_SetYawSpeed(int16_t speedSps)
{
    g_gimbalAttitude.snapshot.commandSps = speedSps;
    if (speedSps > 0) {
        Motor_Set(MOTOR_GIMBAL_1, MOTOR_FORWARD, (uint16_t)speedSps);
    } else if (speedSps < 0) {
        Motor_Set(MOTOR_GIMBAL_1, MOTOR_REVERSE,
            (uint16_t)(-(int32_t)speedSps));
    } else {
        Motor_Set(MOTOR_GIMBAL_1, MOTOR_COAST, 0U);
    }
}

static void GimbalAttitude_ClearReference(void)
{
    g_gimbalAttitude.snapshot.hasReference = 0U;
    g_gimbalAttitude.snapshot.referenceStep =
        Motor_GetStepCount(MOTOR_GIMBAL_1);
    g_gimbalAttitude.snapshot.currentStep =
        g_gimbalAttitude.snapshot.referenceStep;
    g_gimbalAttitude.snapshot.stepError = 0;
    g_gimbalAttitude.snapshot.feedForwardSps = 0;
}

static void GimbalAttitude_CaptureReference(void)
{
    g_gimbalAttitude.baseStep = Motor_GetStepCount(MOTOR_GIMBAL_1);
    g_gimbalAttitude.baseYawX100 =
        g_gimbalAttitude.snapshot.motion.yawControlX100;
    g_gimbalAttitude.snapshot.referenceStep = g_gimbalAttitude.baseStep;
    g_gimbalAttitude.snapshot.currentStep = g_gimbalAttitude.baseStep;
    g_gimbalAttitude.snapshot.stepError = 0;
    g_gimbalAttitude.snapshot.hasReference = 1U;
}

void GimbalAttitude_Init(void)
{
    g_gimbalAttitude.snapshot.config.stepsPerRevolution =
        (uint16_t)GIMBAL_ATTITUDE_STEPS_PER_REVOLUTION;
    g_gimbalAttitude.snapshot.config.directionSign =
        (int8_t)GIMBAL_ATTITUDE_DIRECTION_SIGN;
    g_gimbalAttitude.snapshot.config.kffQ1024 =
        (uint16_t)GIMBAL_ATTITUDE_KFF_Q1024;
    g_gimbalAttitude.snapshot.config.kpQ1024 =
        (uint16_t)GIMBAL_ATTITUDE_KP_Q1024;
    g_gimbalAttitude.snapshot.config.maxSpeedSps =
        (uint16_t)GIMBAL_ATTITUDE_MAX_SPEED_SPS;
    g_gimbalAttitude.snapshot.config.accelStepSps =
        (uint16_t)GIMBAL_ATTITUDE_ACCEL_STEP_SPS;
    g_gimbalAttitude.snapshot.config.positionLimitSteps =
        (uint32_t)GIMBAL_ATTITUDE_POSITION_LIMIT_STEPS;
    g_gimbalAttitude.snapshot.active = 0U;
    g_gimbalAttitude.snapshot.holdEnabled = 0U;
    g_gimbalAttitude.snapshot.feedForwardEnabled = 0U;
    GimbalAttitude_ClearReference();
}

void GimbalAttitude_Start(void)
{
    g_gimbalAttitude.snapshot.active = 0U;
    Motor_Set(MOTOR_GIMBAL_1, MOTOR_COAST, 0U);
    Motor_SetRampStep(MOTOR_GIMBAL_1,
        g_gimbalAttitude.snapshot.config.accelStepSps,
        g_gimbalAttitude.snapshot.config.accelStepSps);
    g_gimbalAttitude.snapshot.holdEnabled = 1U;
    g_gimbalAttitude.snapshot.feedForwardEnabled = 0U;
    GimbalAttitude_ClearReference();
    BodyMotion_StartCalibration();
    g_gimbalAttitude.snapshot.active = 1U;
}

void GimbalAttitude_Stop(void)
{
    g_gimbalAttitude.snapshot.active = 0U;
    GimbalAttitude_SetYawSpeed(0);
    g_gimbalAttitude.snapshot.holdEnabled = 0U;
    g_gimbalAttitude.snapshot.feedForwardEnabled = 0U;
    GimbalAttitude_ClearReference();
    Motor_ResetRampStep(MOTOR_GIMBAL_1);
}

void GimbalAttitude_StartCalibration(void)
{
    uint8_t wasActive = g_gimbalAttitude.snapshot.active;

    g_gimbalAttitude.snapshot.active = 0U;
    GimbalAttitude_SetYawSpeed(0);
    GimbalAttitude_ClearReference();
    BodyMotion_StartCalibration();
    g_gimbalAttitude.snapshot.active = wasActive;
}

void GimbalAttitude_SetHoldEnabled(uint8_t enabled)
{
    g_gimbalAttitude.snapshot.holdEnabled = (enabled != 0U) ? 1U : 0U;
    GimbalAttitude_SetYawSpeed(0);
    GimbalAttitude_ClearReference();
}

void GimbalAttitude_SetFeedForwardEnabled(uint8_t enabled)
{
    g_gimbalAttitude.snapshot.feedForwardEnabled =
        (enabled != 0U) ? 1U : 0U;
    if (enabled == 0U) {
        g_gimbalAttitude.snapshot.feedForwardSps = 0;
    }
}

void GimbalAttitude_Task(void)
{
    int32_t yawDeltaX100;
    int32_t referenceDelta;
    int32_t feedForward;
    int32_t proportional;
    int32_t command;
    int32_t lowerLimit;
    int32_t upperLimit;

    if (g_gimbalAttitude.snapshot.active == 0U) {
        return;
    }
    BodyMotion_GetSnapshot(&g_gimbalAttitude.snapshot.motion);
    if (g_gimbalAttitude.snapshot.motion.state != BODY_MOTION_READY) {
        GimbalAttitude_SetYawSpeed(0);
        GimbalAttitude_ClearReference();
        return;
    }
    if (g_gimbalAttitude.snapshot.holdEnabled == 0U) {
        GimbalAttitude_SetYawSpeed(0);
        return;
    }
    if (g_gimbalAttitude.snapshot.hasReference == 0U) {
        GimbalAttitude_CaptureReference();
        GimbalAttitude_SetYawSpeed(0);
        return;
    }

    yawDeltaX100 = g_gimbalAttitude.snapshot.motion.yawControlX100 -
        g_gimbalAttitude.baseYawX100;
    referenceDelta = GimbalAttitude_DivideRounded(
        (int64_t)yawDeltaX100 *
            g_gimbalAttitude.snapshot.config.stepsPerRevolution *
            g_gimbalAttitude.snapshot.config.directionSign,
        GIMBAL_ATTITUDE_TURN_X100);
    if (g_gimbalAttitude.snapshot.config.positionLimitSteps != 0U) {
        lowerLimit = GimbalAttitude_ClampInt32((int64_t)g_gimbalAttitude.baseStep -
            g_gimbalAttitude.snapshot.config.positionLimitSteps);
        upperLimit = GimbalAttitude_ClampInt32((int64_t)g_gimbalAttitude.baseStep +
            g_gimbalAttitude.snapshot.config.positionLimitSteps);
        if (referenceDelta > (upperLimit - g_gimbalAttitude.baseStep)) {
            referenceDelta = upperLimit - g_gimbalAttitude.baseStep;
        } else if (referenceDelta <
            (lowerLimit - g_gimbalAttitude.baseStep)) {
            referenceDelta = lowerLimit - g_gimbalAttitude.baseStep;
        }
    }
    g_gimbalAttitude.snapshot.referenceStep =
        GimbalAttitude_ClampInt32((int64_t)g_gimbalAttitude.baseStep +
            referenceDelta);
    g_gimbalAttitude.snapshot.currentStep =
        Motor_GetStepCount(MOTOR_GIMBAL_1);
    g_gimbalAttitude.snapshot.stepError =
        g_gimbalAttitude.snapshot.referenceStep -
            g_gimbalAttitude.snapshot.currentStep;

    feedForward = 0;
    if (g_gimbalAttitude.snapshot.feedForwardEnabled != 0U) {
        feedForward = GimbalAttitude_DivideRounded(
            (int64_t)g_gimbalAttitude.snapshot.motion.
                yawRateFilteredX100PerSec *
                g_gimbalAttitude.snapshot.config.stepsPerRevolution *
                g_gimbalAttitude.snapshot.config.directionSign,
            GIMBAL_ATTITUDE_TURN_X100);
        feedForward = GimbalAttitude_DivideRounded(
            (int64_t)feedForward *
                g_gimbalAttitude.snapshot.config.kffQ1024,
            GIMBAL_ATTITUDE_Q1024_SCALE);
    }
    proportional = GimbalAttitude_DivideRounded(
        (int64_t)g_gimbalAttitude.snapshot.stepError *
            g_gimbalAttitude.snapshot.config.kpQ1024,
        GIMBAL_ATTITUDE_Q1024_SCALE);
    command = feedForward + proportional;

    if (g_gimbalAttitude.snapshot.config.positionLimitSteps != 0U) {
        lowerLimit = GimbalAttitude_ClampInt32((int64_t)g_gimbalAttitude.baseStep -
            g_gimbalAttitude.snapshot.config.positionLimitSteps);
        upperLimit = GimbalAttitude_ClampInt32((int64_t)g_gimbalAttitude.baseStep +
            g_gimbalAttitude.snapshot.config.positionLimitSteps);
        if (((g_gimbalAttitude.snapshot.currentStep <= lowerLimit) &&
            (command < 0)) ||
            ((g_gimbalAttitude.snapshot.currentStep >= upperLimit) &&
            (command > 0))) {
            command = 0;
        }
    }
    g_gimbalAttitude.snapshot.feedForwardSps =
        GimbalAttitude_ClampSpeed(feedForward);
    GimbalAttitude_SetYawSpeed(GimbalAttitude_ClampSpeed(command));
}

uint8_t GimbalAttitude_IsActive(void)
{
    return g_gimbalAttitude.snapshot.active;
}

void GimbalAttitude_GetSnapshot(GimbalAttitudeSnapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }
    taskENTER_CRITICAL();
    *snapshot = g_gimbalAttitude.snapshot;
    taskEXIT_CRITICAL();
}

void GimbalAttitude_GetConfig(GimbalAttitudeConfig *config)
{
    if (config != 0) {
        taskENTER_CRITICAL();
        *config = g_gimbalAttitude.snapshot.config;
        taskEXIT_CRITICAL();
    }
}

uint8_t GimbalAttitude_SetConfig(const GimbalAttitudeConfig *config)
{
    uint8_t wasActive;

    if ((config == 0) || (config->stepsPerRevolution < 200U) ||
        (config->directionSign == 0) ||
        ((config->directionSign != 1) && (config->directionSign != -1)) ||
        (config->kffQ1024 > 4096U) || (config->kpQ1024 > 8192U) ||
        (config->maxSpeedSps == 0U) ||
        (config->maxSpeedSps > CAR_STEPPER_SPEED_MAX_SPS) ||
        (config->accelStepSps == 0U) ||
        (config->accelStepSps > CAR_STEPPER_SPEED_MAX_SPS)) {
        return 0U;
    }
    taskENTER_CRITICAL();
    wasActive = g_gimbalAttitude.snapshot.active;
    g_gimbalAttitude.snapshot.active = 0U;
    g_gimbalAttitude.snapshot.config = *config;
    taskEXIT_CRITICAL();
    Motor_SetRampStep(MOTOR_GIMBAL_1, config->accelStepSps,
        config->accelStepSps);
    GimbalAttitude_SetYawSpeed(0);
    GimbalAttitude_ClearReference();
    g_gimbalAttitude.snapshot.active = wasActive;
    return 1U;
}
