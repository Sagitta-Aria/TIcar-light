#include "board.h"

#include "encoder.h"
#include "gray.h"
#include "jq8400.h"
#include "jy61p.h"
#include "key.h"
#include "link.h"
#include "motor.h"
#include "oled.h"
#include "ti_msp_dl_config.h"

void Board_Init(void)
{
    SYSCFG_DL_init();

    Motor_Init();
    Gray_Init();
    Encoder_Init();
    Key_Init();
    JY61P_Init();
    JQ8400_Init();
    Link_Init();

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();
}

void Board_Task(void)
{
}
