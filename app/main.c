#include "app.h"
#include "board.h"
#include "board_config.h"
#include "rtos_app.h"

int main(void)
{
    Board_Init();

#if (CAR_RECOVERY_SAFE_BUILD != 0U)
    for (;;) {
        Board_Task();
    }
#else
    if (Board_HasFatalError() != 0U) {
        for (;;) {
            Board_Task();
        }
    }
    App_Init();
    RtosApp_StartScheduler();
#endif

    return 0;
}
