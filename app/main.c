#include "app.h"
#include "board.h"
#include "board_config.h"

int main(void)
{
    Board_Init();
#if (CAR_RECOVERY_SAFE_BUILD == 0U)
    if (Board_HasFatalError() == 0U) {
        App_Init();
    }
#endif

    while (1) {
        Board_Task();
#if (CAR_RECOVERY_SAFE_BUILD == 0U)
        if (Board_HasFatalError() == 0U) {
            App_Task();
        }
#endif
    }
}
