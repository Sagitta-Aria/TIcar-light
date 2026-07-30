#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "app.h"

#include "board.h"
#include "car_display.h"
#include "delay.h"
#include "h7_control_uart.h"
#include "key.h"
#include "m0_attitude_link.h"
#include "m0_attitude_uart.h"
#include "menu.h"
#include "resource_config.h"
#include "rtos_app.h"

static uint8_t g_inputHadEvent;

static CarEvent App_HandleKeyEvent(KeyEvent event)
{
    CarState state;

    if (event == KEY_EVENT_NONE) {
        return CAR_EVENT_NONE;
    }
    state = StateMachine_GetState();
    if (event == KEY_EVENT_3) {
        if (state == CAR_STATE_MENU) {
            (void)Menu_Back();
            return CAR_EVENT_NONE;
        }
        return (state == CAR_STATE_MISSION) ? CAR_EVENT_STOP : CAR_EVENT_MENU;
    }
    if ((state == CAR_STATE_MENU) && (event == KEY_EVENT_1)) {
        Menu_Next();
        return CAR_EVENT_NONE;
    }
    if ((state == CAR_STATE_MENU) && (event == KEY_EVENT_4)) {
        Menu_Previous();
        return CAR_EVENT_NONE;
    }
    if ((state == CAR_STATE_MENU) && (event == KEY_EVENT_2)) {
        return Menu_Confirm();
    }
    if ((state == CAR_STATE_MISSION) &&
        (StateMachine_GetMissionId() == 5U)) {
        if (event == KEY_EVENT_1) {
            return CAR_EVENT_MISSION_4_MODE_TOGGLE;
        }
        if (event == KEY_EVENT_2) {
            return CAR_EVENT_MISSION_4_RUN_TOGGLE;
        }
    }
    return CAR_EVENT_NONE;
}

void App_Init(void)
{
    Board_ShowBootProgress("GMR Tianmeng", "H7 UART2", "M0 UART3",
        "APP...", "");
    M0AttitudeLink_Init();
#if CAR_M0_ATTITUDE_UART_REQUIRED
    M0AttitudeUart_Init();
#endif
    Menu_Init();
    StateMachine_Init();
    CarDisplay_Clear();
    Menu_Task(StateMachine_GetState());
}

CarEvent App_InputStep(void)
{
    KeyEvent keyEvent;

    Key_Task();
    keyEvent = Key_PopEvent();
    g_inputHadEvent = (keyEvent != KEY_EVENT_NONE) ? 1U : 0U;
    return App_HandleKeyEvent(keyEvent);
}

uint8_t App_InputHadEvent(void)
{
    return g_inputHadEvent;
}

void App_CommStep(void)
{
    H7ControlCommand command;

#if CAR_M0_ATTITUDE_UART_REQUIRED
    M0AttitudeUart_Task(5U);
#endif
    command = H7ControlUart_TakeCommand();
    if (command == H7_CONTROL_COMMAND_START_ATTITUDE) {
        (void)RtosApp_PostEvent(CAR_EVENT_MISSION_1_START);
    } else if (command == H7_CONTROL_COMMAND_START_GRAY_FOLLOW) {
        (void)RtosApp_PostEvent(CAR_EVENT_MISSION_2_START);
    } else if (command == H7_CONTROL_COMMAND_START_ENCODER) {
        (void)RtosApp_PostEvent(CAR_EVENT_MISSION_3_START);
    } else if (command == H7_CONTROL_COMMAND_START_DRIVE) {
        (void)RtosApp_PostEvent(CAR_EVENT_MISSION_4_START);
    } else if (command == H7_CONTROL_COMMAND_START_DIRECTION) {
        (void)RtosApp_PostEvent(CAR_EVENT_MISSION_5_START);
    } else if (command == H7_CONTROL_COMMAND_START_GRAY) {
        (void)RtosApp_PostEvent(CAR_EVENT_MISSION_6_START);
    } else if (command == H7_CONTROL_COMMAND_START_LINE_FOLLOW) {
        (void)RtosApp_PostEvent(CAR_EVENT_MISSION_7_START);
    } else if (command == H7_CONTROL_COMMAND_STOP) {
        (void)RtosApp_PostEvent(CAR_EVENT_STOP);
    }
}

void App_GimbalStep(void)
{
}

void App_VisionInputStep(void)
{
}

void App_UiStep(void)
{
    Menu_Task(StateMachine_GetState());
}

uint8_t App_HousekeepingStep(void)
{
    return Board_Task();
}

void App_Task(void)
{
    CarEvent event = App_InputStep();

    App_MissionDispatch(event);
    App_MissionStep();
    App_CommStep();
    App_UiStep();
    (void)App_HousekeepingStep();
    delay_ms(CAR_APP_LOOP_DELAY_MS);
}

#endif
