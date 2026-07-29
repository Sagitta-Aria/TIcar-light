#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "state_machine.h"

#include "control_config.h"
#include "encoder_motor.h"
#include "gmr_bluetooth_mission.h"
#include "motor_no_yaw.h"
#include "rtos_app.h"
#include "tuning_console.h"

static CarState g_carState = CAR_STATE_INIT;
static uint8_t g_missionId;
static CarChassisDriveMode g_driveMode;
static uint16_t g_driveSpeed;

static uint16_t StateMachine_ClampDriveSpeed(uint16_t speed)
{
    if (speed < (uint16_t)CHASSIS_DEBUG_SPEED_MIN) {
        return (uint16_t)CHASSIS_DEBUG_SPEED_MIN;
    }
    if (speed > (uint16_t)CHASSIS_DEBUG_SPEED_MAX) {
        return (uint16_t)CHASSIS_DEBUG_SPEED_MAX;
    }
    return speed;
}

static int16_t StateMachine_PercentToPwm(uint16_t percent)
{
    return (int16_t)(((uint32_t)CHASSIS_PWM_LIMIT_COUNTS * percent) /
        100U);
}

static void StateMachine_StopRuntime(void)
{
    GmrBluetoothMission_Stop();
    MotorNoYaw_Stop();
    TuningConsole_Stop();
    EncoderMotor_Stop();
}

static void StateMachine_StartMission(void)
{
    StateMachine_StopRuntime();
    if (g_missionId == 3U) {
        EncoderMotor_ResetAllCounts();
    } else if (g_missionId == 4U) {
        MotorNoYaw_StartMission4();
    } else if (g_missionId == 2U) {
        TuningConsole_Start();
    } else if (g_missionId == 5U) {
        TuningConsole_StartYaw();
    } else if (g_missionId == 1U) {
        if (g_driveMode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) {
            EncoderMotor_SetPeriodTargets((int16_t)g_driveSpeed,
                (int16_t)g_driveSpeed);
        } else {
            int16_t pwm = StateMachine_PercentToPwm(g_driveSpeed);

            EncoderMotor_SetOpenLoopPwm(pwm, pwm);
        }
    } else if (g_missionId == 6U) {
        GmrBluetoothMission_Start();
    }
}

static void StateMachine_Enter(CarState nextState)
{
    if (g_carState == nextState) {
        return;
    }
    if (nextState != CAR_STATE_MISSION) {
        StateMachine_StopRuntime();
    }
    g_carState = nextState;
    if (nextState == CAR_STATE_MENU) {
        g_missionId = 0U;
    } else if (nextState == CAR_STATE_MISSION) {
        StateMachine_StartMission();
    }
    RtosApp_NotifyMission();
    RtosApp_NotifyUi();
}

void StateMachine_Init(void)
{
    g_carState = CAR_STATE_INIT;
    g_missionId = 0U;
    g_driveMode = (CHASSIS_TASK1_DEFAULT_CLOSED_LOOP != 0U) ?
        CAR_CHASSIS_DRIVE_CLOSED_LOOP : CAR_CHASSIS_DRIVE_OPEN_LOOP;
    g_driveSpeed = (g_driveMode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ?
        (uint16_t)CHASSIS_TASK1_CLOSED_SPEED_DEFAULT :
        (uint16_t)CHASSIS_TASK1_OPEN_SPEED_DEFAULT;
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
    } else if ((g_carState == CAR_STATE_MENU) &&
        ((event == CAR_EVENT_MISSION_1_START) ||
            (event == CAR_EVENT_MISSION_2_START) ||
            (event == CAR_EVENT_MISSION_3_START) ||
            (event == CAR_EVENT_MISSION_4_START) ||
            (event == CAR_EVENT_MISSION_5_START) ||
            (event == CAR_EVENT_MISSION_6_START))) {
        if (event == CAR_EVENT_MISSION_1_START) {
            g_missionId = 1U;
        } else if (event == CAR_EVENT_MISSION_2_START) {
            g_missionId = 2U;
        } else if (event == CAR_EVENT_MISSION_3_START) {
            g_missionId = 3U;
        } else if (event == CAR_EVENT_MISSION_4_START) {
            g_missionId = 4U;
        } else {
            g_missionId = (event == CAR_EVENT_MISSION_5_START) ? 5U : 6U;
        }
        StateMachine_Enter(CAR_STATE_MISSION);
    }
}

void StateMachine_Task(void)
{
}

void StateMachine_ChassisControlPeriod(void)
{
    GmrBluetoothMissionResult bluetoothResult;

    if (g_carState != CAR_STATE_MISSION) {
        return;
    }
    if (g_missionId == 4U) {
        MotorNoYaw_Task();
        if ((MotorNoYaw_IsRunning() == 0U) &&
            (MotorNoYaw_GetState() == MOTOR_NO_YAW_STATE_STOP)) {
            StateMachine_Enter(CAR_STATE_STOP);
        }
    } else if ((g_missionId == 2U) || (g_missionId == 5U)) {
        TuningConsole_ChassisControlPeriod();
    } else if (g_missionId == 6U) {
        bluetoothResult = GmrBluetoothMission_ControlPeriod();
        if (bluetoothResult == GMR_BLUETOOTH_MISSION_RESULT_FINISHED) {
            StateMachine_Enter(CAR_STATE_FINISHED);
        } else if (bluetoothResult ==
            GMR_BLUETOOTH_MISSION_RESULT_ERROR) {
            StateMachine_Enter(CAR_STATE_ERROR);
        }
    }
}

void StateMachine_HandleChassisFastEvent(void)
{
    if ((g_carState == CAR_STATE_MISSION) && (g_missionId == 4U)) {
        MotorNoYaw_HandleFastEvent();
    } else if ((g_carState == CAR_STATE_MISSION) &&
        (g_missionId == 6U)) {
        GmrBluetoothMission_HandleFastEvent();
    }
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

uint8_t StateMachine_IsMissionAvailable(uint8_t missionId)
{
    return (uint8_t)(((missionId == 1U) || (missionId == 2U) ||
        (missionId == 3U) ||
        (missionId == 4U) ||
        (missionId == 5U) ||
        (missionId == 6U)) ? 1U : 0U);
}

void StateMachine_SetMission1LapCount(uint8_t lapCount)
{
    (void)lapCount;
}

uint8_t StateMachine_GetMission1LapCount(void)
{
    return 1U;
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
    if (missionId != 1U) {
        return;
    }
    g_driveMode = (mode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ?
        CAR_CHASSIS_DRIVE_CLOSED_LOOP : CAR_CHASSIS_DRIVE_OPEN_LOOP;
    g_driveSpeed = StateMachine_ClampDriveSpeed(speed);
}

CarChassisDriveMode StateMachine_GetMissionDriveMode(uint8_t missionId)
{
    return (missionId == 1U) ? g_driveMode : CAR_CHASSIS_DRIVE_OPEN_LOOP;
}

uint16_t StateMachine_GetMissionDriveSpeed(uint8_t missionId)
{
    return (missionId == 1U) ? g_driveSpeed : 0U;
}

CarMission4Stage StateMachine_GetMission4Stage(void)
{
    return ((g_missionId == 4U) && (MotorNoYaw_IsRunning() != 0U)) ?
        CAR_MISSION4_STAGE_LINE :
        CAR_MISSION4_STAGE_IDLE;
}

uint32_t StateMachine_GetMission4Flag(void)
{
    return MotorNoYaw_GetTurnCount();
}

uint8_t StateMachine_IsMissionGimbalPrepDone(void)
{
    return 1U;
}

#endif
