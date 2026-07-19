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
    int32_t travelOriginStep;
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

static int32_t GimbalAttitude_ApplyQ1024(int32_t value,
    uint16_t gainQ1024)
{
    return GimbalAttitude_DivideRounded((int64_t)value * gainQ1024,
        GIMBAL_ATTITUDE_Q1024_SCALE);
}

static int32_t GimbalAttitude_ApplyDeadband(int32_t value,
    int32_t deadband)
{
    if (value > deadband) {
        return value - deadband;
    }
    if (value < -deadband) {
        return value + deadband;
    }
    return 0;
}

static int32_t GimbalAttitude_ClampRate(int32_t rateX100PerSec)
{
    int32_t limit = GimbalAttitude_DivideRounded(
        (int64_t)g_gimbalAttitude.snapshot.config.maxSpeedSps *
            GIMBAL_ATTITUDE_TURN_X100,
        g_gimbalAttitude.snapshot.config.stepsPerRevolution);

    if (rateX100PerSec > limit) {
        return limit;
    }
    if (rateX100PerSec < -limit) {
        return -limit;
    }
    return rateX100PerSec;
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

static int16_t GimbalAttitude_RateToSpeed(int32_t rateX100PerSec)
{
    int32_t speed = GimbalAttitude_DivideRounded(
        (int64_t)rateX100PerSec *
            g_gimbalAttitude.snapshot.config.stepsPerRevolution *
            g_gimbalAttitude.snapshot.config.motorDirectionSign,
        GIMBAL_ATTITUDE_TURN_X100);

    return GimbalAttitude_ClampSpeed(speed);
}

static void GimbalAttitude_SetYawSpeed(int16_t speedSps)
{
    g_gimbalAttitude.snapshot.commandSps = speedSps;
    if (g_gimbalAttitude.snapshot.directOutput == 0U) {
        return;
    }
    if (speedSps > 0) {
        Motor_Set(MOTOR_GIMBAL_1, MOTOR_FORWARD, (uint16_t)speedSps);
    } else if (speedSps < 0) {
        Motor_Set(MOTOR_GIMBAL_1, MOTOR_REVERSE,
            (uint16_t)(-(int32_t)speedSps));
    } else {
        Motor_Set(MOTOR_GIMBAL_1, MOTOR_COAST, 0U);
        g_gimbalAttitude.snapshot.stepOutputSps = 0;
    }
}

static void GimbalAttitude_ClearControl(void)
{
    g_gimbalAttitude.snapshot.hasReference = 0U;
    g_gimbalAttitude.snapshot.referenceYawX100 =
        g_gimbalAttitude.snapshot.feedbackYawX100;
    g_gimbalAttitude.snapshot.angleErrorX100 = 0;
    g_gimbalAttitude.snapshot.rateReferenceX100PerSec = 0;
    g_gimbalAttitude.snapshot.rateErrorX100PerSec = 0;
    g_gimbalAttitude.snapshot.feedForwardSps = 0;
    g_gimbalAttitude.snapshot.rateFeedbackSps = 0;
}

static void GimbalAttitude_CaptureReference(void)
{
    g_gimbalAttitude.snapshot.referenceYawX100 =
        g_gimbalAttitude.snapshot.feedbackYawX100;
    g_gimbalAttitude.snapshot.angleErrorX100 = 0;
    g_gimbalAttitude.snapshot.rateReferenceX100PerSec = 0;
    g_gimbalAttitude.snapshot.rateErrorX100PerSec = 0;
    g_gimbalAttitude.snapshot.hasReference = 1U;
}

static uint8_t GimbalAttitude_UpdateFeedback(uint32_t now)
{
    if (H7GyroLink_GetFeedback(&g_gimbalAttitude.snapshot.feedback) == 0U) {
        g_gimbalAttitude.snapshot.feedbackAngleAgeMs = 0xFFFFFFFFUL;
        g_gimbalAttitude.snapshot.feedbackGyroAgeMs = 0xFFFFFFFFUL;
        g_gimbalAttitude.snapshot.feedbackFresh = 0U;
        return 0U;
    }

    g_gimbalAttitude.snapshot.feedbackAngleAgeMs =
        (g_gimbalAttitude.snapshot.feedback.angleFrameCount == 0U) ?
        0xFFFFFFFFUL : now -
            g_gimbalAttitude.snapshot.feedback.angleFrameTick;
    g_gimbalAttitude.snapshot.feedbackGyroAgeMs =
        (g_gimbalAttitude.snapshot.feedback.gyroFrameCount == 0U) ?
        0xFFFFFFFFUL : now -
            g_gimbalAttitude.snapshot.feedback.gyroFrameTick;
    g_gimbalAttitude.snapshot.feedbackFresh = (uint8_t)(
        (g_gimbalAttitude.snapshot.feedbackAngleAgeMs <=
            H7_GYRO_FEEDBACK_STALE_MS) &&
        (g_gimbalAttitude.snapshot.feedbackGyroAgeMs <=
            H7_GYRO_FEEDBACK_STALE_MS));
    if (g_gimbalAttitude.snapshot.feedbackFresh == 0U) {
        return 0U;
    }

    g_gimbalAttitude.snapshot.feedbackYawX100 =
        GimbalAttitude_ClampInt32(
            (int64_t)g_gimbalAttitude.snapshot.feedback.yawUnwrappedX100 *
                g_gimbalAttitude.snapshot.config.h7FeedbackSign);
    g_gimbalAttitude.snapshot.feedbackRateX100PerSec =
        GimbalAttitude_ClampInt32(
            (int64_t)g_gimbalAttitude.snapshot.feedback.
                yawRateX100PerSec *
                g_gimbalAttitude.snapshot.config.h7FeedbackSign);
    return 1U;
}

void GimbalAttitude_Init(void)
{
    g_gimbalAttitude.snapshot.config.stepsPerRevolution =
        (uint16_t)GIMBAL_ATTITUDE_STEPS_PER_REVOLUTION;
    g_gimbalAttitude.snapshot.config.motorDirectionSign =
        (int8_t)GIMBAL_ATTITUDE_MOTOR_DIRECTION_SIGN;
    g_gimbalAttitude.snapshot.config.h7FeedbackSign =
        (int8_t)GIMBAL_ATTITUDE_H7_FEEDBACK_SIGN;
    g_gimbalAttitude.snapshot.config.jy61FeedForwardSign =
        (int8_t)GIMBAL_ATTITUDE_JY61_FEEDFORWARD_SIGN;
    g_gimbalAttitude.snapshot.config.jy61KffQ1024 =
        (uint16_t)GIMBAL_ATTITUDE_JY61_KFF_Q1024;
    g_gimbalAttitude.snapshot.config.h7AngleKpQ1024 =
        (uint16_t)GIMBAL_ATTITUDE_H7_ANGLE_KP_Q1024;
    g_gimbalAttitude.snapshot.config.h7RateKpQ1024 =
        (uint16_t)GIMBAL_ATTITUDE_H7_RATE_KP_Q1024;
    g_gimbalAttitude.snapshot.config.maxSpeedSps =
        (uint16_t)GIMBAL_ATTITUDE_MAX_SPEED_SPS;
    g_gimbalAttitude.snapshot.config.accelStepSps =
        (uint16_t)GIMBAL_ATTITUDE_ACCEL_STEP_SPS;
    g_gimbalAttitude.snapshot.config.positionLimitSteps =
        (uint32_t)GIMBAL_ATTITUDE_POSITION_LIMIT_STEPS;
    g_gimbalAttitude.snapshot.active = 0U;
    g_gimbalAttitude.snapshot.holdEnabled = 0U;
    g_gimbalAttitude.snapshot.feedForwardEnabled = 0U;
    g_gimbalAttitude.snapshot.feedbackFresh = 0U;
    g_gimbalAttitude.snapshot.feedForwardFresh = 0U;
    g_gimbalAttitude.snapshot.feedbackYawX100 = 0;
    g_gimbalAttitude.snapshot.feedbackRateX100PerSec = 0;
    g_gimbalAttitude.snapshot.currentStep =
        Motor_GetStepCount(MOTOR_GIMBAL_1);
    g_gimbalAttitude.snapshot.stepOutputSps = 0;
    g_gimbalAttitude.snapshot.referenceTracking = 0U;
    g_gimbalAttitude.snapshot.directOutput = 0U;
    g_gimbalAttitude.snapshot.feedbackAngleAgeMs = 0xFFFFFFFFUL;
    g_gimbalAttitude.snapshot.feedbackGyroAgeMs = 0xFFFFFFFFUL;
    g_gimbalAttitude.travelOriginStep =
        g_gimbalAttitude.snapshot.currentStep;
    GimbalAttitude_ClearControl();
}

static void GimbalAttitude_StartInternal(uint8_t directOutput)
{
    uint8_t wasDirect = g_gimbalAttitude.snapshot.directOutput;

    g_gimbalAttitude.snapshot.active = 0U;
    if (wasDirect != 0U) {
        Motor_Set(MOTOR_GIMBAL_1, MOTOR_COAST, 0U);
        Motor_ResetRampStep(MOTOR_GIMBAL_1);
    }
    g_gimbalAttitude.snapshot.directOutput =
        (directOutput != 0U) ? 1U : 0U;
    if (g_gimbalAttitude.snapshot.directOutput != 0U) {
        Motor_Set(MOTOR_GIMBAL_1, MOTOR_COAST, 0U);
        Motor_SetRampStep(MOTOR_GIMBAL_1,
            g_gimbalAttitude.snapshot.config.accelStepSps,
            g_gimbalAttitude.snapshot.config.accelStepSps);
    }
    g_gimbalAttitude.snapshot.holdEnabled = 1U;
    g_gimbalAttitude.snapshot.feedForwardEnabled = 0U;
    g_gimbalAttitude.snapshot.commandSps = 0;
    g_gimbalAttitude.snapshot.stepOutputSps = 0;
    g_gimbalAttitude.snapshot.referenceTracking = 0U;
    g_gimbalAttitude.snapshot.currentStep =
        Motor_GetStepCount(MOTOR_GIMBAL_1);
    g_gimbalAttitude.travelOriginStep =
        g_gimbalAttitude.snapshot.currentStep;
    GimbalAttitude_ClearControl();
    g_gimbalAttitude.snapshot.active = 1U;
}

void GimbalAttitude_Start(void)
{
    GimbalAttitude_StartInternal(1U);
}

void GimbalAttitude_StartAssist(void)
{
    GimbalAttitude_StartInternal(0U);
}

void GimbalAttitude_Stop(void)
{
    uint8_t wasDirect = g_gimbalAttitude.snapshot.directOutput;

    g_gimbalAttitude.snapshot.active = 0U;
    GimbalAttitude_SetYawSpeed(0);
    g_gimbalAttitude.snapshot.holdEnabled = 0U;
    g_gimbalAttitude.snapshot.feedForwardEnabled = 0U;
    g_gimbalAttitude.snapshot.feedForwardFresh = 0U;
    g_gimbalAttitude.snapshot.stepOutputSps = 0;
    g_gimbalAttitude.snapshot.referenceTracking = 0U;
    GimbalAttitude_ClearControl();
    if (wasDirect != 0U) {
        Motor_ResetRampStep(MOTOR_GIMBAL_1);
    }
    g_gimbalAttitude.snapshot.directOutput = 0U;
}

void GimbalAttitude_StartCalibration(void)
{
    uint8_t wasActive = g_gimbalAttitude.snapshot.active;

    g_gimbalAttitude.snapshot.active = 0U;
    GimbalAttitude_SetYawSpeed(0);
    GimbalAttitude_ClearControl();
    BodyMotion_StartCalibration();
    g_gimbalAttitude.snapshot.active = wasActive;
}

void GimbalAttitude_SetHoldEnabled(uint8_t enabled)
{
    g_gimbalAttitude.snapshot.holdEnabled = (enabled != 0U) ? 1U : 0U;
    GimbalAttitude_SetYawSpeed(0);
    GimbalAttitude_ClearControl();
}

void GimbalAttitude_SetFeedForwardEnabled(uint8_t enabled)
{
    g_gimbalAttitude.snapshot.feedForwardEnabled =
        (enabled != 0U) ? 1U : 0U;
    if (enabled == 0U) {
        g_gimbalAttitude.snapshot.feedForwardSps = 0;
    }
}

void GimbalAttitude_SetReferenceTracking(uint8_t enabled)
{
    uint8_t nextEnabled = (enabled != 0U) ? 1U : 0U;

    if (nextEnabled == g_gimbalAttitude.snapshot.referenceTracking) {
        return;
    }
    g_gimbalAttitude.snapshot.referenceTracking = nextEnabled;
    GimbalAttitude_ClearControl();
}

void GimbalAttitude_Task(void)
{
    uint32_t now;
    int32_t feedbackRateX100PerSec;
    int32_t feedForwardRateX100PerSec = 0;
    int32_t rateCorrectionX100PerSec;
    int32_t commandRateX100PerSec;
    int32_t lowerLimit;
    int32_t upperLimit;
    int16_t commandSps;

    if (g_gimbalAttitude.snapshot.active == 0U) {
        return;
    }
    now = (uint32_t)xTaskGetTickCount();
    g_gimbalAttitude.snapshot.currentStep =
        Motor_GetStepCount(MOTOR_GIMBAL_1);
    g_gimbalAttitude.snapshot.stepOutputSps =
        Motor_GetGimbalStepRate(MOTOR_GIMBAL_1);
    BodyMotion_GetSnapshot(&g_gimbalAttitude.snapshot.motion);
    g_gimbalAttitude.snapshot.feedForwardFresh = (uint8_t)(
        (g_gimbalAttitude.snapshot.motion.state == BODY_MOTION_READY) ?
        1U : 0U);

    if (GimbalAttitude_UpdateFeedback(now) == 0U) {
        GimbalAttitude_SetYawSpeed(0);
        GimbalAttitude_ClearControl();
        return;
    }
    if (g_gimbalAttitude.snapshot.holdEnabled == 0U) {
        GimbalAttitude_SetYawSpeed(0);
        GimbalAttitude_ClearControl();
        return;
    }
    if (g_gimbalAttitude.snapshot.referenceTracking != 0U) {
        /* 主动视觉/搜索期间角度参考跟随，但仍继续计算底座yaw前馈补偿。 */
        GimbalAttitude_CaptureReference();
    } else if (g_gimbalAttitude.snapshot.hasReference == 0U) {
        GimbalAttitude_CaptureReference();
        GimbalAttitude_SetYawSpeed(0);
        return;
    }

    g_gimbalAttitude.snapshot.angleErrorX100 =
        g_gimbalAttitude.snapshot.referenceYawX100 -
            g_gimbalAttitude.snapshot.feedbackYawX100;
    g_gimbalAttitude.snapshot.angleErrorX100 =
        GimbalAttitude_ApplyDeadband(
            g_gimbalAttitude.snapshot.angleErrorX100,
            GIMBAL_ATTITUDE_H7_ANGLE_DEADBAND_X100);
    g_gimbalAttitude.snapshot.rateReferenceX100PerSec =
        GimbalAttitude_ClampRate(GimbalAttitude_ApplyQ1024(
            g_gimbalAttitude.snapshot.angleErrorX100,
            g_gimbalAttitude.snapshot.config.h7AngleKpQ1024));

    feedbackRateX100PerSec =
        g_gimbalAttitude.snapshot.feedbackRateX100PerSec;
    g_gimbalAttitude.snapshot.rateErrorX100PerSec =
        g_gimbalAttitude.snapshot.rateReferenceX100PerSec -
            feedbackRateX100PerSec;
    rateCorrectionX100PerSec = GimbalAttitude_ClampRate(
        GimbalAttitude_ApplyQ1024(
            g_gimbalAttitude.snapshot.rateErrorX100PerSec,
            g_gimbalAttitude.snapshot.config.h7RateKpQ1024));

    if ((g_gimbalAttitude.snapshot.feedForwardEnabled != 0U) &&
        (g_gimbalAttitude.snapshot.feedForwardFresh != 0U)) {
        feedForwardRateX100PerSec = GimbalAttitude_ApplyQ1024(
            GimbalAttitude_ClampInt32(
                (int64_t)g_gimbalAttitude.snapshot.motion.
                    yawRateFilteredX100PerSec *
                    g_gimbalAttitude.snapshot.config.jy61FeedForwardSign),
            g_gimbalAttitude.snapshot.config.jy61KffQ1024);
    }

    commandRateX100PerSec = GimbalAttitude_ClampInt32(
        (int64_t)g_gimbalAttitude.snapshot.rateReferenceX100PerSec +
            rateCorrectionX100PerSec - feedForwardRateX100PerSec);
    g_gimbalAttitude.snapshot.feedForwardSps =
        GimbalAttitude_RateToSpeed(-feedForwardRateX100PerSec);
    g_gimbalAttitude.snapshot.rateFeedbackSps =
        GimbalAttitude_RateToSpeed(rateCorrectionX100PerSec);
    commandSps = GimbalAttitude_RateToSpeed(commandRateX100PerSec);

    if (g_gimbalAttitude.snapshot.config.positionLimitSteps != 0U) {
        lowerLimit = GimbalAttitude_ClampInt32(
            (int64_t)g_gimbalAttitude.travelOriginStep -
                g_gimbalAttitude.snapshot.config.positionLimitSteps);
        upperLimit = GimbalAttitude_ClampInt32(
            (int64_t)g_gimbalAttitude.travelOriginStep +
                g_gimbalAttitude.snapshot.config.positionLimitSteps);
        if (((g_gimbalAttitude.snapshot.currentStep <= lowerLimit) &&
            (commandSps < 0)) ||
            ((g_gimbalAttitude.snapshot.currentStep >= upperLimit) &&
            (commandSps > 0))) {
            commandSps = 0;
        }
    }
    GimbalAttitude_SetYawSpeed(commandSps);
}

uint8_t GimbalAttitude_IsActive(void)
{
    return g_gimbalAttitude.snapshot.active;
}

uint8_t GimbalAttitude_DrivesMotorDirectly(void)
{
    return g_gimbalAttitude.snapshot.directOutput;
}

int16_t GimbalAttitude_GetCommandSps(void)
{
    return g_gimbalAttitude.snapshot.commandSps;
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
        ((config->motorDirectionSign != 1) &&
            (config->motorDirectionSign != -1)) ||
        ((config->h7FeedbackSign != 1) &&
            (config->h7FeedbackSign != -1)) ||
        ((config->jy61FeedForwardSign != 1) &&
            (config->jy61FeedForwardSign != -1)) ||
        (config->jy61KffQ1024 > 4096U) ||
        (config->h7AngleKpQ1024 > 32768U) ||
        (config->h7RateKpQ1024 > 4096U) ||
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
    if (g_gimbalAttitude.snapshot.directOutput != 0U) {
        Motor_SetRampStep(MOTOR_GIMBAL_1, config->accelStepSps,
            config->accelStepSps);
    }
    GimbalAttitude_SetYawSpeed(0);
    g_gimbalAttitude.travelOriginStep =
        Motor_GetStepCount(MOTOR_GIMBAL_1);
    GimbalAttitude_ClearControl();
    g_gimbalAttitude.snapshot.active = wasActive;
    return 1U;
}
