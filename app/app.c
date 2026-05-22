#include "app.h"

#include "board.h"
#include "board_config.h"
#include "delay.h"
#include "key.h"
#include "link.h"
#include "menu.h"
#include "motor.h"
#include "motor_test.h"
#include "oled.h"
#include "route.h"
#include "speed_control.h"
#include "state_machine.h"
#include "tracking.h"

#define APP_PB8_LED_DEBOUNCE_TICKS  (3U)

/*
 * 作用：轮询 PB8，并在稳定按下时翻转板载 LED。
 * 使用场景：临时排查 PB8 按键是否真的被 MCU 读到；不依赖 GPIO 中断事件。
 */
static void App_DebugPb8LedTask(void)
{
    static uint8_t initialized;
    static uint8_t lastRawLevel;
    static uint8_t stableLevel;
    static uint8_t stableTicks;
    uint8_t rawLevel;

    rawLevel = Key_IsPressed(KEY_ID_2);
    if (initialized == 0U) {
        initialized = 1U;
        lastRawLevel = rawLevel;
        stableLevel = rawLevel;
        stableTicks = APP_PB8_LED_DEBOUNCE_TICKS;
        return;
    }

    if (rawLevel != lastRawLevel) {
        lastRawLevel = rawLevel;
        stableTicks = 0U;
        return;
    }

    if (stableTicks < APP_PB8_LED_DEBOUNCE_TICKS) {
        ++stableTicks;
        if (stableTicks < APP_PB8_LED_DEBOUNCE_TICKS) {
            return;
        }
    }

    if (rawLevel != stableLevel) {
        stableLevel = rawLevel;
        if (stableLevel != 0U) {
            Board_DebugLedToggle();
        }
    }
}

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
    Board_ShowBootProgress("CLK OK", "I2C OK", "UART OK", "ADC OK", "APP...");
    delay_ms(100U);

    Tracking_Init();
    Route_Init();
    SpeedControl_Init();
    MotorTest_Init();
    Menu_Init();
    Link_SendString("light-car1.1ccs init ok\r\n");
    StateMachine_Init();

    Board_ShowBootProgress("CLK OK", "I2C OK", "UART OK", "ADC OK", "APP OK");
    delay_ms(200U);

    OLED_Clear();
    Menu_Task(StateMachine_GetState());
}

void App_Task(void)
{
    App_DebugPb8LedTask();
    App_HandleKeyEvent(Key_PopEvent());

    StateMachine_Task();

    /*
     * 电机方向测试需要直接输出 STEP，不能被速度闭环覆盖。
     * 1.1ccs 默认关闭编码器速度闭环，正式循迹仍走统一的左右轮命令接口。
     */
    if (StateMachine_GetState() != CAR_STATE_MOTOR_TEST) {
        SpeedControl_Task();
    }

    Menu_Task(StateMachine_GetState());
    Link_Task();
    Motor_Task();
    delay_ms(CAR_APP_LOOP_DELAY_MS);
}
