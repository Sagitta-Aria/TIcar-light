#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "state_machine.h"

#include "control_config.h"
#include "encoder_motor.h"
#include "gray.h"
#include "gmr_stop_count_store.h"
#include "h7_control_uart.h"
#include "rtos_app.h"
#include "FreeRTOS.h"
#include "task.h"

#if !CAR_LIBRARY_GRAY_INPUT_IS_INFRARED_8
#error "GMR Task2 and infrared test require eight-channel infrared input"
#endif

#if (GRAY_SENSOR_COUNT != 8U)
#error "GMR weighted line following requires exactly eight sensors"
#endif

typedef enum {
    GMR_TASK45_RAMP_IDLE = 0,
    GMR_TASK45_RAMP_ACCELERATING,
    GMR_TASK45_RAMP_CRUISING,
    GMR_TASK45_RAMP_DECELERATING
} GmrTask45RampPhase;

static CarState g_carState = CAR_STATE_INIT;
static uint8_t g_missionId;

static uint16_t g_mission4LeftTarget;
static uint16_t g_mission4RightTarget;
static CarChassisDriveMode g_mission5DriveMode;
static uint16_t g_mission5DriveSpeed;
static TickType_t g_taskTimerStartTick;
static uint32_t g_taskTimerElapsedMs;
static uint8_t g_taskTimerRunning;
static int32_t g_mission9TargetPosition;
static int32_t g_rampTiltTestPositionUnits;
static int16_t g_rampTiltTestSpeedTarget;
static TickType_t g_task45RampPhaseStartTick;
static int16_t g_task45DecelStartBaseTarget;
static GmrTask45RampPhase g_task45RampPhase;
static int16_t g_lineFollowLastRawLeftTarget;
static int16_t g_lineFollowLastRawRightTarget;
static uint32_t g_lineFollowLostElapsedMs;
static uint8_t g_lineFollowHasLastTarget;

typedef struct {
    int16_t baseSpeed;
    int16_t minimumSpeed;
    int16_t maximumSpeed;
    int16_t gain;
    int16_t correctionLimit;
    uint16_t lostHoldMs;
    int16_t grayWeights[GRAY_SENSOR_COUNT];
} GmrLineFollowConfig;

static int16_t StateMachine_ScaleLineFollowTarget(int16_t target,
    int16_t baseTarget, const GmrLineFollowConfig *config);

static const GmrLineFollowConfig g_task1LineFollow = {
    GMR_TASK1_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD,
    GMR_TASK1_LINE_FOLLOW_MIN_SPEED_COUNTS_PER_PERIOD,
    GMR_TASK1_LINE_FOLLOW_MAX_SPEED_COUNTS_PER_PERIOD,
    GMR_TASK1_LINE_FOLLOW_GAIN,
    GMR_TASK1_LINE_FOLLOW_CORRECTION_LIMIT_COUNTS,
    GMR_TASK1_LINE_FOLLOW_LOST_HOLD_MS,
    {
        GMR_TASK1_LINE_FOLLOW_S1_WEIGHT,
        GMR_TASK1_LINE_FOLLOW_S2_WEIGHT,
        GMR_TASK1_LINE_FOLLOW_S3_WEIGHT,
        GMR_TASK1_LINE_FOLLOW_S4_WEIGHT,
        GMR_TASK1_LINE_FOLLOW_S5_WEIGHT,
        GMR_TASK1_LINE_FOLLOW_S6_WEIGHT,
        GMR_TASK1_LINE_FOLLOW_S7_WEIGHT,
        GMR_TASK1_LINE_FOLLOW_S8_WEIGHT
    }
};

static const GmrLineFollowConfig g_task4LineFollow = {
    GMR_TASK4_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD,
    GMR_TASK4_LINE_FOLLOW_MIN_SPEED_COUNTS_PER_PERIOD,
    GMR_TASK4_LINE_FOLLOW_MAX_SPEED_COUNTS_PER_PERIOD,
    GMR_TASK4_LINE_FOLLOW_GAIN,
    GMR_TASK4_LINE_FOLLOW_CORRECTION_LIMIT_COUNTS,
    GMR_TASK4_LINE_FOLLOW_LOST_HOLD_MS,
    {
        GMR_TASK4_LINE_FOLLOW_S1_WEIGHT,
        GMR_TASK4_LINE_FOLLOW_S2_WEIGHT,
        GMR_TASK4_LINE_FOLLOW_S3_WEIGHT,
        GMR_TASK4_LINE_FOLLOW_S4_WEIGHT,
        GMR_TASK4_LINE_FOLLOW_S5_WEIGHT,
        GMR_TASK4_LINE_FOLLOW_S6_WEIGHT,
        GMR_TASK4_LINE_FOLLOW_S7_WEIGHT,
        GMR_TASK4_LINE_FOLLOW_S8_WEIGHT
    }
};

static const GmrLineFollowConfig g_task5LineFollow = {
    GMR_TASK5_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD,
    GMR_TASK5_LINE_FOLLOW_MIN_SPEED_COUNTS_PER_PERIOD,
    GMR_TASK5_LINE_FOLLOW_MAX_SPEED_COUNTS_PER_PERIOD,
    GMR_TASK5_LINE_FOLLOW_GAIN,
    GMR_TASK5_LINE_FOLLOW_CORRECTION_LIMIT_COUNTS,
    GMR_TASK5_LINE_FOLLOW_LOST_HOLD_MS,
    {
        GMR_TASK5_LINE_FOLLOW_S1_WEIGHT,
        GMR_TASK5_LINE_FOLLOW_S2_WEIGHT,
        GMR_TASK5_LINE_FOLLOW_S3_WEIGHT,
        GMR_TASK5_LINE_FOLLOW_S4_WEIGHT,
        GMR_TASK5_LINE_FOLLOW_S5_WEIGHT,
        GMR_TASK5_LINE_FOLLOW_S6_WEIGHT,
        GMR_TASK5_LINE_FOLLOW_S7_WEIGHT,
        GMR_TASK5_LINE_FOLLOW_S8_WEIGHT
    }
};

static int16_t StateMachine_ClampLineFollowTarget(int32_t target,
    const GmrLineFollowConfig *config)
{
    if (target > config->maximumSpeed) {
        target = config->maximumSpeed;
    } else if (target < config->minimumSpeed) {
        target = config->minimumSpeed;
    }
    return (int16_t)target;
}

static void StateMachine_ResetLineFollowMemory(void)
{
    g_lineFollowLastRawLeftTarget = 0;
    g_lineFollowLastRawRightTarget = 0;
    g_lineFollowLostElapsedMs = 0U;
    g_lineFollowHasLastTarget = 0U;
}

/* 丢线时沿用最后一次有效灰度比例，并继续服从当前纵向速度斜坡。 */
static void StateMachine_ApplyLostLineTarget(int16_t baseTarget,
    const GmrLineFollowConfig *config)
{
    if ((g_lineFollowHasLastTarget != 0U) &&
        (g_lineFollowLostElapsedMs < config->lostHoldMs)) {
        EncoderMotor_SetPeriodTargets(
            StateMachine_ScaleLineFollowTarget(
                g_lineFollowLastRawLeftTarget, baseTarget, config),
            StateMachine_ScaleLineFollowTarget(
                g_lineFollowLastRawRightTarget, baseTarget, config));
        g_lineFollowLostElapsedMs += CHASSIS_CONTROL_PERIOD_MS;
        return;
    }
    EncoderMotor_SetPeriodTargets(0, 0);
}

static void StateMachine_ApplyMission5Drive(void)
{
    uint32_t pwm;

    EncoderMotor_Stop();
    if (g_mission5DriveSpeed == 0U) {
        return;
    }
    if (g_mission5DriveMode == CAR_CHASSIS_DRIVE_OPEN_LOOP) {
        pwm = ((uint32_t)g_mission5DriveSpeed *
            (uint32_t)CHASSIS_PWM_LIMIT_COUNTS + 50U) / 100U;
        EncoderMotor_SetOpenLoopPwm((int16_t)pwm, (int16_t)pwm);
    } else {
        EncoderMotor_SetPeriodTargets((int16_t)g_mission5DriveSpeed,
            (int16_t)g_mission5DriveSpeed);
    }
}

static int16_t StateMachine_ScaleLineFollowTarget(int16_t target,
    int16_t baseTarget, const GmrLineFollowConfig *config)
{
    int32_t scaledTarget;

    if (baseTarget >= config->baseSpeed) {
        return target;
    }
    scaledTarget = ((int32_t)target * (int32_t)baseTarget +
        (config->baseSpeed / 2)) / config->baseSpeed;
    return (int16_t)scaledTarget;
}

static void StateMachine_UpdateLineFollowDrive(int16_t baseTarget,
    const GmrLineFollowConfig *config)
{
    uint8_t grayMask;
    uint8_t activeCount = 0U;
    uint8_t index;
    int32_t weightSum = 0;
    int32_t lineError;
    int32_t correction;
    int16_t leftTarget;
    int16_t rightTarget;

    if (Gray_Update() == 0U) {
        EncoderMotor_SetPeriodTargets(0, 0);
        return;
    }
    grayMask = Gray_GetDigitalMask();
    for (index = 0U; index < GRAY_SENSOR_COUNT; ++index) {
        uint8_t bit = (uint8_t)(1U <<
            ((GRAY_SENSOR_COUNT - 1U) - index));

        if ((grayMask & bit) != 0U) {
            weightSum += config->grayWeights[index];
            ++activeCount;
        }
    }
    if (activeCount == 0U) {
        StateMachine_ApplyLostLineTarget(baseTarget, config);
        return;
    }

    lineError = weightSum / (int32_t)activeCount;
    correction = lineError * config->gain;
    if (correction > config->correctionLimit) {
        correction = config->correctionLimit;
    } else if (correction < -config->correctionLimit) {
        correction = -config->correctionLimit;
    }
    leftTarget = StateMachine_ClampLineFollowTarget(
        config->baseSpeed - correction, config);
    rightTarget = StateMachine_ClampLineFollowTarget(
        config->baseSpeed + correction, config);
    g_lineFollowLastRawLeftTarget = leftTarget;
    g_lineFollowLastRawRightTarget = rightTarget;
    leftTarget = StateMachine_ScaleLineFollowTarget(leftTarget,
        baseTarget, config);
    rightTarget = StateMachine_ScaleLineFollowTarget(rightTarget,
        baseTarget, config);
    g_lineFollowLostElapsedMs = 0U;
    g_lineFollowHasLastTarget = 1U;
    EncoderMotor_SetPeriodTargets(leftTarget, rightTarget);
}

static void StateMachine_StopRuntime(CarState nextState)
{
    uint8_t task456Mission = (uint8_t)(
        ((g_missionId == CAR_MISSION_ID_GMR_TASK4_TRACK_BALL) ||
        (g_missionId == CAR_MISSION_ID_GMR_TASK5_TRACK_BALL) ||
        (g_missionId == CAR_MISSION_ID_GMR_TASK6_TRACK_BALL)) ? 1U : 0U);
    uint8_t h7BallMission = (uint8_t)(
        ((g_missionId == 8U) || (g_missionId == 9U) ||
        (g_missionId == CAR_MISSION_ID_GMR_TASK3_BALL) ||
        (task456Mission != 0U) ||
        (g_missionId == CAR_MISSION_ID_GMR_IMU_Y_FF_TEST)) ? 1U : 0U);
    uint8_t keepTask456BallBalance = (uint8_t)(
        ((nextState == CAR_STATE_FINISHED) &&
        (g_carState == CAR_STATE_MISSION) &&
        (task456Mission != 0U)) ? 1U : 0U);

    EncoderMotor_Stop();
    if ((g_carState == CAR_STATE_MISSION) &&
        (g_missionId == CAR_MISSION_ID_GMR_RAMP_TILT_TEST)) {
        H7ControlUart_RequestStepperMove(0L,
            GMR_RAMP_TILT_TEST_STEPPER_SPEED_RPM,
            GMR_RAMP_TILT_TEST_STEPPER_ACCELERATION);
    } else if ((keepTask456BallBalance == 0U) &&
        (((g_carState == CAR_STATE_MISSION) &&
        (h7BallMission != 0U)) ||
        ((g_carState == CAR_STATE_FINISHED) &&
        (task456Mission != 0U)))) {
        H7ControlUart_RequestBallBalanceStop();
    }
    StateMachine_ResetLineFollowMemory();
    g_task45RampPhase = GMR_TASK45_RAMP_IDLE;
}

static uint8_t StateMachine_AreEncoderThresholdsReached(
    GmrStopCountItem leftItem, GmrStopCountItem rightItem,
    uint32_t leadCounts)
{
    int32_t leftCount;
    int32_t rightCount;
    uint32_t leftStopCount = GmrStopCountStore_GetValue(leftItem);
    uint32_t rightStopCount = GmrStopCountStore_GetValue(rightItem);
    uint32_t leftThreshold = (leftStopCount > leadCounts) ?
        (leftStopCount - leadCounts) : (leftStopCount / 2U);
    uint32_t rightThreshold = (rightStopCount > leadCounts) ?
        (rightStopCount - leadCounts) : (rightStopCount / 2U);

    EncoderMotor_GetTotalCounts(&leftCount, &rightCount);
    return (uint8_t)(((leftCount > (int32_t)leftThreshold) &&
        (rightCount > (int32_t)rightThreshold)) ? 1U : 0U);
}

static uint32_t StateMachine_GetTaskRunningElapsedMs(void)
{
    TickType_t elapsedTicks = xTaskGetTickCount() - g_taskTimerStartTick;

    return (uint32_t)pdTICKS_TO_MS(elapsedTicks);
}

/* 按Task1起步段的同一整数斜率计算任意时刻的共同速度目标。 */
static int16_t StateMachine_GetMission2RampTargetAtMs(uint32_t elapsedMs)
{
    uint32_t speedRange;
    uint32_t rampSpeed;

    speedRange = (uint32_t)(
        GMR_TASK1_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD -
            GMR_MISSION2_START_SPEED_COUNTS_PER_PERIOD);
    rampSpeed = (uint32_t)GMR_MISSION2_START_SPEED_COUNTS_PER_PERIOD +
        ((speedRange * elapsedMs) / GMR_MISSION2_START_RAMP_MS);
    return (int16_t)rampSpeed;
}

/* Task2起步时从约三分之一基础速度线性升到正常循迹速度。 */
static int16_t StateMachine_GetMission2RampBaseTarget(void)
{
    uint32_t elapsedMs = StateMachine_GetTaskRunningElapsedMs();

    if (elapsedMs >= GMR_MISSION2_START_RAMP_MS) {
        return GMR_TASK1_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD;
    }
    return StateMachine_GetMission2RampTargetAtMs(elapsedMs);
}

static int16_t StateMachine_GetTask45RampBaseTarget(
    const GmrLineFollowConfig *config)
{
    uint32_t elapsedMs = (uint32_t)pdTICKS_TO_MS(
        xTaskGetTickCount() - g_task45RampPhaseStartTick);
    uint32_t rampDurationMs;
    uint32_t speedDelta;

    if (g_task45RampPhase == GMR_TASK45_RAMP_ACCELERATING) {
        rampDurationMs = (((uint32_t)config->baseSpeed * 1000U) +
            GMR_TASK45_LINE_FOLLOW_ACCELERATION_UNITS_PER_SECOND - 1U) /
            GMR_TASK45_LINE_FOLLOW_ACCELERATION_UNITS_PER_SECOND;
        if (elapsedMs >= rampDurationMs) {
            g_task45RampPhase = GMR_TASK45_RAMP_CRUISING;
            return config->baseSpeed;
        }
        speedDelta =
            (GMR_TASK45_LINE_FOLLOW_ACCELERATION_UNITS_PER_SECOND *
                elapsedMs) / 1000U;
        return (int16_t)speedDelta;
    }
    if (g_task45RampPhase == GMR_TASK45_RAMP_DECELERATING) {
        rampDurationMs =
            (((uint32_t)g_task45DecelStartBaseTarget * 1000U) +
            GMR_TASK45_LINE_FOLLOW_DECELERATION_UNITS_PER_SECOND - 1U) /
            GMR_TASK45_LINE_FOLLOW_DECELERATION_UNITS_PER_SECOND;
        if (elapsedMs >= rampDurationMs) {
            return 0;
        }
        speedDelta =
            (GMR_TASK45_LINE_FOLLOW_DECELERATION_UNITS_PER_SECOND *
                elapsedMs) / 1000U;
        return (int16_t)(g_task45DecelStartBaseTarget -
            (int16_t)speedDelta);
    }
    return (g_task45RampPhase == GMR_TASK45_RAMP_CRUISING) ?
        config->baseSpeed : 0;
}

static void StateMachine_StartTask45Deceleration(int16_t baseTarget)
{
    g_task45DecelStartBaseTarget = baseTarget;
    g_task45RampPhaseStartTick = xTaskGetTickCount();
    g_task45RampPhase = GMR_TASK45_RAMP_DECELERATING;
}

static void StateMachine_StopTaskTimer(void)
{
    if (g_taskTimerRunning == 0U) {
        return;
    }
    g_taskTimerElapsedMs = StateMachine_GetTaskRunningElapsedMs();
    g_taskTimerRunning = 0U;
}

static void StateMachine_StartMission(uint8_t missionId)
{
    StateMachine_StopTaskTimer();
    StateMachine_StopRuntime(CAR_STATE_MISSION);
    if ((StateMachine_IsTaskMenuMission(missionId) != 0U) ||
        (missionId == CAR_MISSION_ID_GMR_RAMP_TILT_TEST)) {
        g_taskTimerElapsedMs = 0U;
        g_taskTimerStartTick = xTaskGetTickCount();
        g_taskTimerRunning = 1U;
    }
    if (missionId == 2U) {
        EncoderMotor_ResetAllCounts();
    } else if (missionId == 3U) {
        EncoderMotor_ResetAllCounts();
    } else if (missionId == 4U) {
        EncoderMotor_SetPeriodTargets(
            (int16_t)g_mission4LeftTarget,
            (int16_t)g_mission4RightTarget);
    } else if (missionId == 5U) {
        g_mission5DriveMode = CAR_CHASSIS_DRIVE_OPEN_LOOP;
        g_mission5DriveSpeed = 0U;
    } else if (missionId == 8U) {
        H7ControlUart_RequestBallBalanceStart();
    } else if (missionId == 9U) {
        /* 先退出视觉滚球控制；第一次K5固定发送正3200步。 */
        H7ControlUart_RequestBallBalanceStop();
        g_mission9TargetPosition = 0L;
    } else if (missionId == CAR_MISSION_ID_GMR_TASK3_BALL) {
        H7ControlUart_RequestBallBalanceMode(
            H7_BALL_BALANCE_MODE_TASK3);
    } else if (missionId == CAR_MISSION_ID_GMR_TASK4_TRACK_BALL) {
        EncoderMotor_ResetAllCounts();
        g_task45RampPhaseStartTick = g_taskTimerStartTick;
        g_task45DecelStartBaseTarget = 0;
        g_task45RampPhase = GMR_TASK45_RAMP_ACCELERATING;
        H7ControlUart_RequestBallBalanceMode(
            H7_BALL_BALANCE_MODE_TASK4);
    } else if (missionId == CAR_MISSION_ID_GMR_TASK5_TRACK_BALL) {
        EncoderMotor_ResetAllCounts();
        g_task45RampPhaseStartTick = g_taskTimerStartTick;
        g_task45DecelStartBaseTarget = 0;
        g_task45RampPhase = GMR_TASK45_RAMP_ACCELERATING;
        H7ControlUart_RequestBallBalanceMode(
            H7_BALL_BALANCE_MODE_TASK5);
    } else if (missionId == CAR_MISSION_ID_GMR_TASK6_TRACK_BALL) {
        EncoderMotor_ResetAllCounts();
        g_task45RampPhaseStartTick = g_taskTimerStartTick;
        g_task45DecelStartBaseTarget = 0;
        g_task45RampPhase = GMR_TASK45_RAMP_ACCELERATING;
        H7ControlUart_RequestBallBalanceMode(
            H7_BALL_BALANCE_MODE_TASK6);
    } else if (missionId == CAR_MISSION_ID_GMR_IMU_Y_FF_TEST) {
        H7ControlUart_RequestBallBalanceMode(
            H7_BALL_BALANCE_MODE_IMU_Y_FEEDFORWARD);
    } else if (missionId == CAR_MISSION_ID_GMR_RAMP_TILT_TEST) {
        EncoderMotor_ResetAllCounts();
        g_rampTiltTestSpeedTarget = 0;
        H7ControlUart_RequestStepperMove(g_rampTiltTestPositionUnits,
            GMR_RAMP_TILT_TEST_STEPPER_SPEED_RPM,
            GMR_RAMP_TILT_TEST_STEPPER_ACCELERATION);
    }
    g_missionId = missionId;
    g_carState = CAR_STATE_MISSION;
    RtosApp_NotifyMission();
    RtosApp_NotifyUi();
}

static void StateMachine_Enter(CarState nextState)
{
    if (g_carState == nextState) {
        return;
    }
    StateMachine_StopTaskTimer();
    StateMachine_StopRuntime(nextState);
    g_carState = nextState;
    if (nextState == CAR_STATE_MENU) {
        g_missionId = 0U;
    }
    RtosApp_NotifyMission();
    RtosApp_NotifyUi();
}

void StateMachine_Init(void)
{
    g_carState = CAR_STATE_INIT;
    g_missionId = 0U;
    g_mission4LeftTarget = GMR_MISSION4_LEFT_SPEED_COUNTS_PER_PERIOD;
    g_mission4RightTarget = GMR_MISSION4_RIGHT_SPEED_COUNTS_PER_PERIOD;
    g_mission5DriveMode = CAR_CHASSIS_DRIVE_OPEN_LOOP;
    g_mission5DriveSpeed = 0U;
    g_taskTimerStartTick = 0U;
    g_taskTimerElapsedMs = 0U;
    g_taskTimerRunning = 0U;
    g_mission9TargetPosition = 0L;
    g_rampTiltTestPositionUnits = GMR_RAMP_TILT_TEST_POSITION_STEPS;
    g_rampTiltTestSpeedTarget = 0;
    g_task45RampPhaseStartTick = 0U;
    g_task45DecelStartBaseTarget = 0;
    g_task45RampPhase = GMR_TASK45_RAMP_IDLE;
    StateMachine_Enter(CAR_STATE_MENU);
}

void StateMachine_Dispatch(CarEvent event)
{
    if (event == CAR_EVENT_ERROR) {
        StateMachine_Enter(CAR_STATE_ERROR);
    } else if (event == CAR_EVENT_STOP) {
        StateMachine_Enter(CAR_STATE_STOP);
    } else if ((event == CAR_EVENT_MENU) ||
        (event == CAR_EVENT_CLEAR_ERROR)) {
        StateMachine_Enter(CAR_STATE_MENU);
    } else if (event == CAR_EVENT_FINISHED) {
        StateMachine_Enter(CAR_STATE_FINISHED);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_1_START)) {
        StateMachine_StartMission(1U);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_2_START)) {
        StateMachine_StartMission(2U);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_3_START)) {
        StateMachine_StartMission(3U);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_4_START)) {
        StateMachine_StartMission(4U);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_5_START)) {
        StateMachine_StartMission(5U);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_6_START)) {
        StateMachine_StartMission(6U);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_7_START)) {
        StateMachine_StartMission(7U);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_8_START)) {
        StateMachine_StartMission(8U);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_9_START)) {
        StateMachine_StartMission(9U);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_10_START)) {
        StateMachine_StartMission(CAR_MISSION_ID_GMR_TASK3_BALL);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_11_START)) {
        StateMachine_StartMission(CAR_MISSION_ID_GMR_TASK4_TRACK_BALL);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_12_START)) {
        StateMachine_StartMission(CAR_MISSION_ID_GMR_TASK5_TRACK_BALL);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_13_START)) {
        StateMachine_StartMission(CAR_MISSION_ID_GMR_IMU_Y_FF_TEST);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_14_START)) {
        StateMachine_StartMission(CAR_MISSION_ID_GMR_RAMP_TILT_TEST);
    } else if ((g_carState != CAR_STATE_ERROR) &&
        (event == CAR_EVENT_MISSION_15_START)) {
        StateMachine_StartMission(CAR_MISSION_ID_GMR_TASK6_TRACK_BALL);
    } else if ((g_carState == CAR_STATE_MISSION) &&
        (g_missionId == 9U) &&
        (event == CAR_EVENT_MISSION_9_POSITION_TOGGLE)) {
        g_mission9TargetPosition = (g_mission9TargetPosition > 0L) ?
            -GMR_MISSION9_POSITION_STEPS : GMR_MISSION9_POSITION_STEPS;
        H7ControlUart_RequestStepperMove(g_mission9TargetPosition,
            GMR_MISSION9_SPEED_RPM, GMR_MISSION9_ACCELERATION);
        RtosApp_NotifyUi();
    } else if ((g_carState == CAR_STATE_MISSION) &&
        (g_missionId == 5U) &&
        (event == CAR_EVENT_MISSION_4_MODE_TOGGLE)) {
        StateMachine_SetMissionDriveConfig(5U,
            (g_mission5DriveMode == CAR_CHASSIS_DRIVE_OPEN_LOOP) ?
                CAR_CHASSIS_DRIVE_CLOSED_LOOP :
                CAR_CHASSIS_DRIVE_OPEN_LOOP,
            0U);
    } else if ((g_carState == CAR_STATE_MISSION) &&
        (g_missionId == 5U) &&
        (event == CAR_EVENT_MISSION_4_RUN_TOGGLE)) {
        uint16_t nextSpeed = 0U;

        if (g_mission5DriveSpeed == 0U) {
            nextSpeed = (g_mission5DriveMode ==
                CAR_CHASSIS_DRIVE_OPEN_LOOP) ?
                GMR_MISSION5_DIRECTION_PWM_PERCENT :
                GMR_MISSION5_DIRECTION_SPEED_COUNTS_PER_PERIOD;
        }
        StateMachine_SetMissionDriveConfig(5U, g_mission5DriveMode,
            nextSpeed);
    }
}

void StateMachine_Task(void)
{
}

void StateMachine_ChassisControlPeriod(void)
{
    const GmrLineFollowConfig *config;
    int16_t baseTarget;

    if (g_carState != CAR_STATE_MISSION) {
        return;
    }
    if (g_missionId == CAR_MISSION_ID_GMR_RAMP_TILT_TEST) {
        uint32_t elapsedMs = StateMachine_GetTaskRunningElapsedMs();

        if (elapsedMs >= GMR_RAMP_TILT_TEST_DURATION_MS) {
            StateMachine_Enter(CAR_STATE_FINISHED);
            return;
        }
        g_rampTiltTestSpeedTarget = (int16_t)(
            (GMR_TASK45_LINE_FOLLOW_ACCELERATION_UNITS_PER_SECOND *
                elapsedMs) / 1000U);
        EncoderMotor_SetPeriodTargets(g_rampTiltTestSpeedTarget,
            g_rampTiltTestSpeedTarget);
        return;
    }
    if (g_missionId == CAR_MISSION_ID_GMR_TASK4_TRACK_BALL) {
        config = &g_task4LineFollow;
    } else if ((g_missionId == CAR_MISSION_ID_GMR_TASK5_TRACK_BALL) ||
        (g_missionId == CAR_MISSION_ID_GMR_TASK6_TRACK_BALL)) {
        config = &g_task5LineFollow;
    } else if ((g_missionId == 2U) || (g_missionId == 6U) ||
        (g_missionId == 7U)) {
        config = &g_task1LineFollow;
    } else {
        return;
    }
    if ((g_missionId == 2U) &&
        (StateMachine_AreEncoderThresholdsReached(
            GMR_STOP_COUNT_TASK1_LEFT,
            GMR_STOP_COUNT_TASK1_RIGHT, 0U) != 0U)) {
        StateMachine_Enter(CAR_STATE_FINISHED);
        return;
    }

    if ((g_missionId == CAR_MISSION_ID_GMR_TASK4_TRACK_BALL) ||
        (g_missionId == CAR_MISSION_ID_GMR_TASK5_TRACK_BALL) ||
        (g_missionId == CAR_MISSION_ID_GMR_TASK6_TRACK_BALL)) {
        baseTarget = StateMachine_GetTask45RampBaseTarget(config);
        if ((g_missionId != CAR_MISSION_ID_GMR_TASK6_TRACK_BALL) &&
            (g_task45RampPhase != GMR_TASK45_RAMP_DECELERATING)) {
            GmrStopCountItem leftItem =
                (g_missionId == CAR_MISSION_ID_GMR_TASK4_TRACK_BALL) ?
                    GMR_STOP_COUNT_TASK4_LEFT : GMR_STOP_COUNT_TASK5_LEFT;
            GmrStopCountItem rightItem =
                (g_missionId == CAR_MISSION_ID_GMR_TASK4_TRACK_BALL) ?
                    GMR_STOP_COUNT_TASK4_RIGHT : GMR_STOP_COUNT_TASK5_RIGHT;

            if (StateMachine_AreEncoderThresholdsReached(leftItem,
                rightItem,
                GMR_TASK45_LINE_FOLLOW_DECEL_LEAD_COUNTS) != 0U) {
                StateMachine_StartTask45Deceleration(baseTarget);
            }
        }
        baseTarget = StateMachine_GetTask45RampBaseTarget(config);
        if ((g_task45RampPhase == GMR_TASK45_RAMP_DECELERATING) &&
            (baseTarget == 0)) {
            StateMachine_Enter(CAR_STATE_FINISHED);
            return;
        }
        StateMachine_UpdateLineFollowDrive(baseTarget, config);
        return;
    }

    baseTarget = config->baseSpeed;
    if (g_missionId == 2U) {
        baseTarget = StateMachine_GetMission2RampBaseTarget();
    }
    StateMachine_UpdateLineFollowDrive(baseTarget, config);
}

void StateMachine_HandleChassisFastEvent(void)
{
}

CarState StateMachine_GetState(void)
{
    return g_carState;
}

const char *StateMachine_GetStateName(CarState state)
{
    switch (state) {
    case CAR_STATE_INIT:
        return "init";
    case CAR_STATE_MENU:
        return "menu";
    case CAR_STATE_MISSION:
        return "mission";
    case CAR_STATE_FINISHED:
        return "finished";
    case CAR_STATE_STOP:
        return "stop";
    case CAR_STATE_ERROR:
        return "error";
    default:
        return "unknown";
    }
}

uint8_t StateMachine_GetMissionId(void)
{
    return g_missionId;
}

uint32_t StateMachine_GetTaskElapsedMs(void)
{
    return (g_taskTimerRunning != 0U) ?
        StateMachine_GetTaskRunningElapsedMs() : g_taskTimerElapsedMs;
}

uint8_t StateMachine_IsTaskMenuMission(uint8_t missionId)
{
    return (uint8_t)(((missionId == 2U) || (missionId == 8U) ||
        (missionId == CAR_MISSION_ID_GMR_TASK3_BALL) ||
        (missionId == CAR_MISSION_ID_GMR_TASK4_TRACK_BALL) ||
        (missionId == CAR_MISSION_ID_GMR_TASK5_TRACK_BALL) ||
        (missionId == CAR_MISSION_ID_GMR_TASK6_TRACK_BALL)) ? 1U : 0U);
}

int32_t StateMachine_GetMission9TargetPosition(void)
{
    return g_mission9TargetPosition;
}

int32_t StateMachine_GetRampTiltTestPosition(void)
{
    return g_rampTiltTestPositionUnits;
}

int16_t StateMachine_GetRampTiltTestSpeedTarget(void)
{
    return g_rampTiltTestSpeedTarget;
}

int32_t StateMachine_GetForwardAccelerationX100(void)
{
    uint32_t rampDurationMs;
    uint32_t speedRange;
    uint32_t accelerationX100;

    if ((g_carState != CAR_STATE_MISSION) ||
        (g_taskTimerRunning == 0U)) {
        return 0;
    }
    if ((g_missionId == CAR_MISSION_ID_GMR_TASK4_TRACK_BALL) ||
        (g_missionId == CAR_MISSION_ID_GMR_TASK5_TRACK_BALL) ||
        (g_missionId == CAR_MISSION_ID_GMR_TASK6_TRACK_BALL)) {
        if (g_task45RampPhase == GMR_TASK45_RAMP_ACCELERATING) {
            return (int32_t)(
                GMR_TASK45_LINE_FOLLOW_ACCELERATION_UNITS_PER_SECOND *
                100U);
        }
        if (g_task45RampPhase == GMR_TASK45_RAMP_DECELERATING) {
            return -(int32_t)(
                GMR_TASK45_LINE_FOLLOW_DECELERATION_UNITS_PER_SECOND *
                100U);
        }
        return 0;
    }
    if (g_missionId == CAR_MISSION_ID_GMR_RAMP_TILT_TEST) {
        return (StateMachine_GetTaskRunningElapsedMs() <
            GMR_RAMP_TILT_TEST_DURATION_MS) ?
            (int32_t)(
                GMR_TASK45_LINE_FOLLOW_ACCELERATION_UNITS_PER_SECOND *
                100U) : 0;
    }
    if (g_missionId == 2U) {
        rampDurationMs = GMR_MISSION2_START_RAMP_MS;
    } else {
        return 0;
    }
    if (StateMachine_GetTaskRunningElapsedMs() >= rampDurationMs) {
        return 0;
    }

    speedRange = (uint32_t)(
        GMR_TASK1_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD -
            GMR_MISSION2_START_SPEED_COUNTS_PER_PERIOD);
    /* Convert a millisecond ramp to x100 speed-units per second. */
    accelerationX100 = ((speedRange * 100U * 1000U) +
        (GMR_MISSION2_START_RAMP_MS / 2U)) /
        GMR_MISSION2_START_RAMP_MS;
    return (int32_t)accelerationX100;
}

uint8_t StateMachine_IsMissionAvailable(uint8_t missionId)
{
    return (uint8_t)(((missionId >= 1U) &&
        (missionId <= CAR_MISSION_ID_GMR_TASK6_TRACK_BALL)) ? 1U : 0U);
}

void StateMachine_SetMission1LapCount(uint8_t lapCount)
{
    (void)lapCount;
}

uint8_t StateMachine_GetMission1LapCount(void)
{
    return 1U;
}

void StateMachine_SetMission4Targets(uint16_t leftTarget,
    uint16_t rightTarget)
{
    if (leftTarget > GMR_MISSION4_SPEED_MAX_COUNTS_PER_PERIOD) {
        leftTarget = GMR_MISSION4_SPEED_MAX_COUNTS_PER_PERIOD;
    } else if (leftTarget < GMR_MISSION4_SPEED_MIN_COUNTS_PER_PERIOD) {
        leftTarget = GMR_MISSION4_SPEED_MIN_COUNTS_PER_PERIOD;
    }
    if (rightTarget > GMR_MISSION4_SPEED_MAX_COUNTS_PER_PERIOD) {
        rightTarget = GMR_MISSION4_SPEED_MAX_COUNTS_PER_PERIOD;
    } else if (rightTarget < GMR_MISSION4_SPEED_MIN_COUNTS_PER_PERIOD) {
        rightTarget = GMR_MISSION4_SPEED_MIN_COUNTS_PER_PERIOD;
    }
    g_mission4LeftTarget = leftTarget;
    g_mission4RightTarget = rightTarget;
}

void StateMachine_SetMission4Route(CarMission4Route route)
{
    (void)route;
}

CarMission4Route StateMachine_GetMission4Route(void)
{
    return CAR_MISSION4_POINT_ONE_LAP;
}

void StateMachine_SetMissionDriveConfig(uint8_t missionId,
    CarChassisDriveMode mode, uint16_t speed)
{
    if ((missionId != 5U) ||
        ((mode != CAR_CHASSIS_DRIVE_OPEN_LOOP) &&
            (mode != CAR_CHASSIS_DRIVE_CLOSED_LOOP))) {
        return;
    }
    g_mission5DriveMode = mode;
    if (speed == 0U) {
        g_mission5DriveSpeed = 0U;
    } else if (mode == CAR_CHASSIS_DRIVE_OPEN_LOOP) {
        g_mission5DriveSpeed = GMR_MISSION5_DIRECTION_PWM_PERCENT;
    } else {
        g_mission5DriveSpeed =
            GMR_MISSION5_DIRECTION_SPEED_COUNTS_PER_PERIOD;
    }
    StateMachine_ApplyMission5Drive();
    RtosApp_NotifyUi();
}

CarChassisDriveMode StateMachine_GetMissionDriveMode(uint8_t missionId)
{
    return (missionId == 5U) ? g_mission5DriveMode :
        CAR_CHASSIS_DRIVE_OPEN_LOOP;
}

uint16_t StateMachine_GetMissionDriveSpeed(uint8_t missionId)
{
    return (missionId == 5U) ? g_mission5DriveSpeed : 0U;
}

CarMission4Stage StateMachine_GetMission4Stage(void)
{
    return CAR_MISSION4_STAGE_IDLE;
}

uint32_t StateMachine_GetMission4Flag(void)
{
    return 0U;
}

uint8_t StateMachine_IsMissionGimbalPrepDone(void)
{
    return 1U;
}

#endif
