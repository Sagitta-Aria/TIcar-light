#include "app.h"

#include "board_config.h"
#include "delay.h"
#include "key.h"
#include "link.h"
#include "motor_test.h"
#include "oled.h"
#include "route.h"
#include "speed_control.h"
#include "state_machine.h"
#include "tracking.h"

void App_Init(void)
{
    Tracking_Init();
    Route_Init();
    SpeedControl_Init();
    MotorTest_Init();
    Link_SendString("light-car1.0 init ok\r\n");
    StateMachine_Init();
    OLED_ShowLine(0U, "light-car1.0", 12U);
    OLED_ShowLine(1U, "OLED OK", 12U);
    OLED_Refresh();
}

void App_Task(void)
{
    KeyEvent event = Key_PopEvent();

    if (event == KEY_EVENT_1) {
#if CAR_ENABLE_MOTOR_TEST_MODE
        StateMachine_Dispatch(CAR_EVENT_MOTOR_TEST_NEXT);
#else
        StateMachine_Dispatch(CAR_EVENT_START);
#endif
    } else if (event == KEY_EVENT_2) {
        StateMachine_Dispatch(CAR_EVENT_STOP);
    }

    StateMachine_Task();

    /*
     * 电机方向测试需要直接输出 PWM，不能被速度闭环覆盖。
     * 正式循迹和普通停车状态仍然由速度闭环统一刷新。
     */
    if (StateMachine_GetState() != CAR_STATE_MOTOR_TEST) {
        SpeedControl_Task();
    }

    Link_Task();
    delay_ms(CAR_APP_LOOP_DELAY_MS);
}
