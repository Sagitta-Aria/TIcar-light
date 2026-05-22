#include "app.h"
#include "board.h"

int main(void)
{
    Board_Init();
    if (Board_HasFatalError() == 0U) {
        App_Init();
    }

    while (1) {
        Board_Task();
        if (Board_HasFatalError() == 0U) {
            App_Task();
        }
    }
}
