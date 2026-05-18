#include "state_machine.h"

#include "link.h"
#include "motion.h"
#include "tracking.h"

static CarState g_carState = CAR_STATE_INIT;

static void StateMachine_Enter(CarState nextState)
{
    if (g_carState == nextState) {
        return;
    }

    g_carState = nextState;

    switch (g_carState) {
    case CAR_STATE_IDLE:
        Tracking_SetEnabled(0U);
        Motion_Stop();
        Link_SendString("state: idle\r\n");
        break;

    case CAR_STATE_TRACKING:
        Tracking_SetEnabled(1U);
        Link_SendString("state: tracking\r\n");
        break;

    case CAR_STATE_STOP:
        Tracking_SetEnabled(0U);
        Motion_Stop();
        Link_SendString("state: stop\r\n");
        break;

    case CAR_STATE_ERROR:
        Tracking_SetEnabled(0U);
        Motion_Stop();
        Link_SendString("state: error\r\n");
        break;

    case CAR_STATE_INIT:
    default:
        Motion_Stop();
        break;
    }
}

void StateMachine_Init(void)
{
    g_carState = CAR_STATE_INIT;
    StateMachine_Enter(CAR_STATE_IDLE);
}

void StateMachine_Dispatch(CarEvent event)
{
    if (event == CAR_EVENT_NONE) {
        return;
    }

    if (event == CAR_EVENT_ERROR) {
        StateMachine_Enter(CAR_STATE_ERROR);
        return;
    }

    switch (g_carState) {
    case CAR_STATE_IDLE:
        if (event == CAR_EVENT_START) {
            StateMachine_Enter(CAR_STATE_TRACKING);
        } else if (event == CAR_EVENT_STOP) {
            StateMachine_Enter(CAR_STATE_STOP);
        }
        break;

    case CAR_STATE_TRACKING:
        if (event == CAR_EVENT_STOP) {
            StateMachine_Enter(CAR_STATE_STOP);
        }
        break;

    case CAR_STATE_STOP:
        if (event == CAR_EVENT_START) {
            StateMachine_Enter(CAR_STATE_TRACKING);
        }
        break;

    case CAR_STATE_ERROR:
        if (event == CAR_EVENT_CLEAR_ERROR) {
            StateMachine_Enter(CAR_STATE_IDLE);
        }
        break;

    case CAR_STATE_INIT:
    default:
        StateMachine_Enter(CAR_STATE_IDLE);
        break;
    }
}

void StateMachine_Task(void)
{
    switch (g_carState) {
    case CAR_STATE_TRACKING:
        Tracking_Task();
        break;

    case CAR_STATE_IDLE:
    case CAR_STATE_STOP:
    case CAR_STATE_ERROR:
    case CAR_STATE_INIT:
    default:
        break;
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
    case CAR_STATE_IDLE:
        return "idle";
    case CAR_STATE_TRACKING:
        return "tracking";
    case CAR_STATE_STOP:
        return "stop";
    case CAR_STATE_ERROR:
        return "error";
    default:
        return "unknown";
    }
}
