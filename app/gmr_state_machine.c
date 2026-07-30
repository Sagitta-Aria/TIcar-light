#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "state_machine.h"

#include "control_config.h"
#include "encoder_motor.h"
#include "gray.h"
#include "rtos_app.h"
#include "FreeRTOS.h"
#include "task.h"

static CarState g_carState = CAR_STATE_INIT;
static uint8_t g_missionId;

static uint16_t g_mission4LeftTarget;
static uint16_t g_mission4RightTarget;
static CarChassisDriveMode g_mission5DriveMode;
static uint16_t g_mission5DriveSpeed;
static TickType_t g_mission2StartTick;
static uint32_t g_mission2ElapsedMs;
static uint8_t g_mission2TimerRunning;
static int16_t g_lineFollowLastLeftTarget;
static int16_t g_lineFollowLastRightTarget;
static uint32_t g_lineFollowLostElapsedMs;
static uint8_t g_lineFollowHasLastTarget;

static const int16_t g_lineFollowGrayWeights[GRAY_SENSOR_COUNT] = {
    GMR_LINE_FOLLOW_S1_WEIGHT,
    GMR_LINE_FOLLOW_S2_WEIGHT,
    GMR_LINE_FOLLOW_S3_WEIGHT,
    GMR_LINE_FOLLOW_S4_WEIGHT,
    GMR_LINE_FOLLOW_S5_WEIGHT,
    GMR_LINE_FOLLOW_S6_WEIGHT,
    GMR_LINE_FOLLOW_S7_WEIGHT,
#if CAR_LIBRARY_GRAY_INPUT_IS_INFRARED_8
    GMR_LINE_FOLLOW_S8_WEIGHT,
#endif
};

static int16_t StateMachine_ClampLineFollowTarget(int32_t target)
{
    if (target > GMR_LINE_FOLLOW_MAX_SPEED_COUNTS_PER_PERIOD) {
        target = GMR_LINE_FOLLOW_MAX_SPEED_COUNTS_PER_PERIOD;
    } else if (target < GMR_LINE_FOLLOW_MIN_SPEED_COUNTS_PER_PERIOD) {
        target = GMR_LINE_FOLLOW_MIN_SPEED_COUNTS_PER_PERIOD;
    }
    return (int16_t)target;
}

static void StateMachine_ResetLineFollowMemory(void)
{
    g_lineFollowLastLeftTarget = 0;
    g_lineFollowLastRightTarget = 0;
    g_lineFollowLostElapsedMs = 0U;
    g_lineFollowHasLastTarget = 0U;
}

/* 丢线时沿用最后一次有效灰度目标，达到保持时间后停车。 */
static void StateMachine_ApplyLostLineTarget(void)
{
    if ((g_lineFollowHasLastTarget != 0U) &&
        (g_lineFollowLostElapsedMs < GMR_LINE_FOLLOW_LOST_HOLD_MS)) {
        EncoderMotor_SetPeriodTargets(g_lineFollowLastLeftTarget,
            g_lineFollowLastRightTarget);
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
    int16_t baseTarget)
{
    int32_t scaledTarget;

    if (baseTarget >= GMR_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD) {
        return target;
    }
    scaledTarget = ((int32_t)target * (int32_t)baseTarget +
        (GMR_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD / 2)) /
        GMR_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD;
    return (int16_t)scaledTarget;
}

static void StateMachine_UpdateLineFollowDrive(int16_t baseTarget)
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
            weightSum += g_lineFollowGrayWeights[index];
            ++activeCount;
        }
    }
    if (activeCount == 0U) {
        StateMachine_ApplyLostLineTarget();
        return;
    }

    lineError = weightSum / (int32_t)activeCount;
    correction = lineError * GMR_LINE_FOLLOW_GAIN;
    if (correction > GMR_LINE_FOLLOW_CORRECTION_LIMIT_COUNTS) {
        correction = GMR_LINE_FOLLOW_CORRECTION_LIMIT_COUNTS;
    } else if (correction < -GMR_LINE_FOLLOW_CORRECTION_LIMIT_COUNTS) {
        correction = -GMR_LINE_FOLLOW_CORRECTION_LIMIT_COUNTS;
    }
    leftTarget = StateMachine_ClampLineFollowTarget(
        GMR_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD - correction);
    rightTarget = StateMachine_ClampLineFollowTarget(
        GMR_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD + correction);
    leftTarget = StateMachine_ScaleLineFollowTarget(leftTarget,
        baseTarget);
    rightTarget = StateMachine_ScaleLineFollowTarget(rightTarget,
        baseTarget);
    g_lineFollowLastLeftTarget = leftTarget;
    g_lineFollowLastRightTarget = rightTarget;
    g_lineFollowLostElapsedMs = 0U;
    g_lineFollowHasLastTarget = 1U;
    EncoderMotor_SetPeriodTargets(leftTarget, rightTarget);
}

static void StateMachine_StopRuntime(void)
{
    EncoderMotor_Stop();
    StateMachine_ResetLineFollowMemory();
}

static uint8_t StateMachine_IsMission2StopReached(void)
{
    int32_t leftCount;
    int32_t rightCount;

    EncoderMotor_GetTotalCounts(&leftCount, &rightCount);
    return (uint8_t)(((leftCount > GMR_MISSION2_LEFT_STOP_COUNT) &&
        (rightCount > GMR_MISSION2_RIGHT_STOP_COUNT)) ? 1U : 0U);
}

static uint32_t StateMachine_GetMission2RunningElapsedMs(void)
{
    TickType_t elapsedTicks = xTaskGetTickCount() - g_mission2StartTick;

    return (uint32_t)pdTICKS_TO_MS(elapsedTicks);
}

/* Task2起步时从约三分之一基础速度线性升到正常循迹速度。 */
static int16_t StateMachine_GetMission2RampBaseTarget(void)
{
    uint32_t elapsedMs = StateMachine_GetMission2RunningElapsedMs();
    uint32_t speedRange;
    uint32_t rampSpeed;

    if (elapsedMs >= GMR_MISSION2_START_RAMP_MS) {
        return GMR_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD;
    }
    speedRange = (uint32_t)(GMR_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD -
        GMR_MISSION2_START_SPEED_COUNTS_PER_PERIOD);
    rampSpeed = (uint32_t)GMR_MISSION2_START_SPEED_COUNTS_PER_PERIOD +
        ((speedRange * elapsedMs) / GMR_MISSION2_START_RAMP_MS);
    return (int16_t)rampSpeed;
}

static void StateMachine_StopMission2Timer(void)
{
    if (g_mission2TimerRunning == 0U) {
        return;
    }
    g_mission2ElapsedMs = StateMachine_GetMission2RunningElapsedMs();
    g_mission2TimerRunning = 0U;
}

static void StateMachine_StartMission(uint8_t missionId)
{
    StateMachine_StopMission2Timer();
    StateMachine_StopRuntime();
    if (missionId == 2U) {
        EncoderMotor_ResetAllCounts();
        g_mission2ElapsedMs = 0U;
        g_mission2StartTick = xTaskGetTickCount();
        g_mission2TimerRunning = 1U;
    } else if (missionId == 3U) {
        EncoderMotor_ResetAllCounts();
    } else if (missionId == 4U) {
        EncoderMotor_SetPeriodTargets(
            (int16_t)g_mission4LeftTarget,
            (int16_t)g_mission4RightTarget);
    } else if (missionId == 5U) {
        g_mission5DriveMode = CAR_CHASSIS_DRIVE_OPEN_LOOP;
        g_mission5DriveSpeed = 0U;
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
    StateMachine_StopMission2Timer();
    StateMachine_StopRuntime();
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
    g_mission2StartTick = 0U;
    g_mission2ElapsedMs = 0U;
    g_mission2TimerRunning = 0U;
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
    int16_t baseTarget = GMR_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD;

    if ((g_carState != CAR_STATE_MISSION) ||
        ((g_missionId != 2U) && (g_missionId != 6U) &&
            (g_missionId != 7U))) {
        return;
    }
    if ((g_missionId == 2U) &&
        (StateMachine_IsMission2StopReached() != 0U)) {
        StateMachine_Enter(CAR_STATE_FINISHED);
        return;
    }
    if (g_missionId == 2U) {
        baseTarget = StateMachine_GetMission2RampBaseTarget();
    }
    StateMachine_UpdateLineFollowDrive(baseTarget);
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

uint32_t StateMachine_GetMission2ElapsedMs(void)
{
    return (g_mission2TimerRunning != 0U) ?
        StateMachine_GetMission2RunningElapsedMs() : g_mission2ElapsedMs;
}

uint8_t StateMachine_IsMissionAvailable(uint8_t missionId)
{
    return (uint8_t)(((missionId >= 1U) && (missionId <= 7U)) ? 1U : 0U);
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
