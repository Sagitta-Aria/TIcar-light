#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "state_machine.h"

#include "encoder_motor.h"
#include "rtos_app.h"

static CarState g_carState = CAR_STATE_INIT;
static uint8_t g_missionId;

static void StateMachine_StopRuntime(void)
{
    EncoderMotor_Stop();
}

static void StateMachine_Enter(CarState nextState)
{
    if (g_carState == nextState) {
        return;
    }
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
        g_missionId = 1U;
        StateMachine_Enter(CAR_STATE_MISSION);
    }
}

void StateMachine_Task(void)
{
}

void StateMachine_ChassisControlPeriod(void)
{
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

uint8_t StateMachine_IsMissionAvailable(uint8_t missionId)
{
    return (uint8_t)((missionId == 1U) ? 1U : 0U);
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
    (void)missionId;
    (void)mode;
    (void)speed;
}

CarChassisDriveMode StateMachine_GetMissionDriveMode(uint8_t missionId)
{
    (void)missionId;
    return CAR_CHASSIS_DRIVE_OPEN_LOOP;
}

uint16_t StateMachine_GetMissionDriveSpeed(uint8_t missionId)
{
    (void)missionId;
    return 0U;
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
