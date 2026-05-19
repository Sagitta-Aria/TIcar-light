#include "app.h"

#include "board_config.h"
#include "delay.h"
#include "key.h"
#include "link.h"
#include "menu.h"
#include "motor_test.h"
#include "oled.h"
#include "route.h"
#include "speed_control.h"
#include "state_machine.h"
#include "tracking.h"

/*
 * 作用：把两个实体按键翻译成菜单/状态机事件。
 * 使用场景：App_Task 每轮取到按键事件后调用。
 * 说明：菜单内 KEY1 为确认、KEY2 为下一个；任务页里 KEY2 作为返回。
 */
static void App_HandleKeyEvent(KeyEvent event)
{
    CarState state;

    if (event == KEY_EVENT_NONE) {
        return;
    }

    state = StateMachine_GetState();
    if (state == CAR_STATE_MENU) {
        if (event == KEY_EVENT_1) {
            StateMachine_Dispatch(Menu_Confirm());
        } else if (event == KEY_EVENT_2) {
            Menu_Next();
        }
        return;
    }

    if (state == CAR_STATE_GRAY_CALIBRATION) {
        if (event == KEY_EVENT_1) {
            StateMachine_Dispatch(CAR_EVENT_GRAY_CALIBRATION_APPLY);
        } else if (event == KEY_EVENT_2) {
            StateMachine_Dispatch(CAR_EVENT_BACK);
        }
        return;
    }

    if (state == CAR_STATE_MOTOR_TEST) {
        if (event == KEY_EVENT_1) {
            StateMachine_Dispatch(CAR_EVENT_MOTOR_TEST_NEXT);
        } else if (event == KEY_EVENT_2) {
            StateMachine_Dispatch(CAR_EVENT_BACK);
        }
        return;
    }

    if (state == CAR_STATE_ERROR) {
        if (event == KEY_EVENT_1) {
            StateMachine_Dispatch(CAR_EVENT_CLEAR_ERROR);
        } else if (event == KEY_EVENT_2) {
            StateMachine_Dispatch(CAR_EVENT_BACK);
        }
        return;
    }

    if ((state == CAR_STATE_STOP) || (state == CAR_STATE_FINISHED) ||
        (state == CAR_STATE_IDLE)) {
        if (event == KEY_EVENT_1) {
            StateMachine_Dispatch(CAR_EVENT_MENU);
        } else if (event == KEY_EVENT_2) {
            StateMachine_Dispatch(CAR_EVENT_BACK);
        }
        return;
    }

    if (event == KEY_EVENT_2) {
        StateMachine_Dispatch(CAR_EVENT_BACK);
    }
}

void App_Init(void)
{
    Tracking_Init();
    Route_Init();
    SpeedControl_Init();
    MotorTest_Init();
    Menu_Init();
    Link_SendString("light-car1.0 init ok\r\n");
    StateMachine_Init();
    Menu_Task(StateMachine_GetState());
}

void App_Task(void)
{
    App_HandleKeyEvent(Key_PopEvent());

    StateMachine_Task();

    /*
     * 电机方向测试需要直接输出 PWM，不能被速度闭环覆盖。
     * 正式循迹和普通停车状态仍然由速度闭环统一刷新。
     */
    if (StateMachine_GetState() != CAR_STATE_MOTOR_TEST) {
        SpeedControl_Task();
    }

    Menu_Task(StateMachine_GetState());
    Link_Task();
    delay_ms(CAR_APP_LOOP_DELAY_MS);
}
