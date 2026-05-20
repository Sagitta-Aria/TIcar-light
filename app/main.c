#include "app.h"
#include "board.h"

int main(void)
{
    Board_Init();
    App_Init();

    while (1) {
        App_Task();
    }
}
