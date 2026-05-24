#include "app.h"

#include "board.h"
#include "board_config.h"
#include "delay.h"
#include "gimbal.h"
#include "gimbal_test.h"
#include "key.h"
#include "link.h"
#include "log_uart.h"
#include "menu.h"
#include "motor.h"
#include "oled.h"
#include "pose_solver.h"
#include "route.h"
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

    /* 按键测试日志：如果一按出现多行，说明硬件抖动或中断消抖还要继续加强。 */
    if (event == KEY_EVENT_1) {
        LOG_LINE("key: K1 PB9");
    } else if (event == KEY_EVENT_2) {
        LOG_LINE("key: K2 PB8");
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
            StateMachine_Dispatch(Menu_GrayCalibrationConfirm());
        } else if (event == KEY_EVENT_2) {
            Menu_GrayCalibrationNext();
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
    Board_ShowBootProgress("I2C OK", "UART OK", "Gray OK", "APP...", "");
    LOG_LINE("app: init begin");
    delay_ms(100U);

    Tracking_Init();
    LOG_LINE("app: tracking init ok");
    Route_Init();
    LOG_LINE("app: route init ok");
    Gimbal_Init();
    LOG_LINE("app: gimbal init ok");
    GimbalTest_Init();
    LOG_LINE("app: gimbal test init ok");
    PoseSolver_Init();
    LOG_LINE("app: pose solver init ok");
    Menu_Init();
    LOG_LINE("app: menu init ok");
    LOG_LINE("light-car ccs1.2 init ok");
    StateMachine_Init();

    Board_ShowBootProgress("I2C OK", "UART OK", "Gray OK", "APP OK", "");
    delay_ms(200U);

    OLED_Clear();
    Menu_Task(StateMachine_GetState());
}

void App_Task(void)
{
    Key_Task();
    App_HandleKeyEvent(Key_PopEvent());

    StateMachine_Task();

    Menu_Task(StateMachine_GetState());
    LogUart_Task();
    Link_Task();
    PoseSolver_Task();
    Gimbal_Task();
    Motor_Task();
    delay_ms(CAR_APP_LOOP_DELAY_MS);
}
