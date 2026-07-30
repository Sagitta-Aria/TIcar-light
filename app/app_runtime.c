#include "app.h"

#include "key.h"

uint8_t App_InputIsActive(void)
{
    return (uint8_t)(((Key_IsPressed(KEY_ID_1) != 0U) ||
        (Key_IsPressed(KEY_ID_2) != 0U) ||
        (Key_IsPressed(KEY_ID_3) != 0U) ||
        (Key_IsPressed(KEY_ID_4) != 0U) ||
        (Key_HasPendingEvent() != 0U)) ? 1U : 0U);
}

void App_MissionDispatch(CarEvent event)
{
    if (event != CAR_EVENT_NONE) {
        StateMachine_Dispatch(event);
    }
}

void App_MissionStep(void)
{
    StateMachine_Task();
}
