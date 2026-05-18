#include "app.h"

#include "board_config.h"
#include "delay.h"
#include "key.h"
#include "link.h"
#include "oled.h"
#include "state_machine.h"
#include "tracking.h"

void App_Init(void)
{
    Tracking_Init();
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
        StateMachine_Dispatch(CAR_EVENT_START);
    } else if (event == KEY_EVENT_2) {
        StateMachine_Dispatch(CAR_EVENT_STOP);
    }

    StateMachine_Task();
    Link_Task();
    delay_ms(CAR_APP_LOOP_DELAY_MS);
}
