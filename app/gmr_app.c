#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "app.h"

#include "board.h"
#include "board_config.h"
#include "bluetooth_service.h"
#include "car_display.h"
#include "delay.h"
#include "gmr_bluetooth_mission.h"
#include "gmr_yaw_control.h"
#include "h7_gyro_link.h"
#if CAR_JY61P_ENABLED
#include "jy61p.h"
#endif
#include "key.h"
#include "log_uart.h"
#include "m0_attitude_link.h"
#include "m0_attitude_uart.h"
#include "menu.h"
#include "motor_no_yaw.h"
#include "resource_config.h"
#include "tuning_console.h"

static uint8_t g_inputHadEvent;

static CarEvent App_HandleKeyEvent(KeyEvent event)
{
    CarState state;

    if (event == KEY_EVENT_NONE) {
        return CAR_EVENT_NONE;
    }
    state = StateMachine_GetState();
    if (event == KEY_EVENT_2_LONG) {
        if (state == CAR_STATE_MENU) {
            (void)Menu_Back();
            return CAR_EVENT_NONE;
        }
        return (state == CAR_STATE_MISSION) ? CAR_EVENT_STOP : CAR_EVENT_MENU;
    }
    if (state == CAR_STATE_MENU) {
        if (event == KEY_EVENT_1) {
            Menu_Next();
        } else if (event == KEY_EVENT_2) {
            return Menu_Confirm();
        }
        return CAR_EVENT_NONE;
    }
    if (((state == CAR_STATE_STOP) || (state == CAR_STATE_FINISHED) ||
        (state == CAR_STATE_ERROR)) && (event == KEY_EVENT_2)) {
        return CAR_EVENT_MENU;
    }
    return CAR_EVENT_NONE;
}

void App_Init(void)
{
    Board_ShowBootProgress("GMR", "UART OK", "Motor OK", "APP...", "");
    BluetoothService_Init();
#if CAR_JY61P_ENABLED
    JY61P_Init();
#endif
#if CAR_LIBRARY_H7_IMU_ENABLED
    H7GyroLink_Init();
#endif
    M0AttitudeLink_Init();
#if CAR_M0_ATTITUDE_UART_REQUIRED
    M0AttitudeUart_Init();
#endif
    GmrYawControl_Init();
    MotorNoYaw_Init();
    GmrBluetoothMission_Init();
    Menu_Init();
    StateMachine_Init();
    CarDisplay_Clear();
    CarDisplay_Refresh();
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
    BluetoothService_Task(5U);
    GmrBluetoothMission_CommTask(5U);
#if CAR_M0_ATTITUDE_UART_REQUIRED
    M0AttitudeUart_Task(5U);
#endif
    GmrYawControl_Observe();
    LogUart_Task();
    TuningConsole_Task();
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
