#include "board.h"

#include "delay.h"
#include "encoder.h"
#include "gray.h"
#include "jq8400.h"
#include "jy61p.h"
#include "key.h"
#include "link.h"
#include "motor.h"
#include "oled.h"
#include "ti_msp_dl_config.h"

/*
 * 作用：把启动阶段的 5 行状态写到 OLED。
 * 使用场景：排查时钟、I2C、UART、ADC、应用层初始化卡在哪一步。
 * 说明：每次都会刷新整屏，所以适合只在开机阶段调用。
 */
static void Board_ShowBootLine(uint8_t line, const char *text)
{
    char buffer[22];
    uint8_t index;

    for (index = 0U; index < 21U; ++index) {
        buffer[index] = ' ';
    }
    buffer[21U] = '\0';

    if (text != NULL) {
        for (index = 0U; (index < 21U) && (text[index] != '\0'); ++index) {
            buffer[index] = text[index];
        }
    }

    OLED_ShowLine(line, buffer, 12U);
}

/*
 * 作用：在上电阶段先点亮 OLED，显示当前初始化进度。
 * 使用场景：排查是否卡在时钟初始化、外设初始化或复位边界。
 */
void Board_ShowBootProgress(const char *clkStatus, const char *i2cStatus,
    const char *uartStatus, const char *adcStatus, const char *appStatus)
{
    Board_ShowBootLine(0U, clkStatus);
    Board_ShowBootLine(1U, i2cStatus);
    Board_ShowBootLine(2U, uartStatus);
    Board_ShowBootLine(3U, adcStatus);
    Board_ShowBootLine(4U, appStatus);
    OLED_Refresh();
}

void Board_Init(void)
{
    /*
     * 先只初始化电源、GPIO 和 OLED。
     * 这样如果系统时钟卡住，屏幕还能停在启动探针页。
     */
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    SYSCFG_DL_OLED_init();
    OLED_Init();
    Board_ShowBootProgress("CLK...", "I2C OK", "UART WAIT", "ADC WAIT",
        "APP WAIT");
    delay_ms(200U);

    /*
     * 再初始化系统时钟。
     * 这里如果 HFXT / SYSPLL 不稳定，屏幕会明显卡在上一页。
     */
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_OLED_init();
    Board_ShowBootProgress("CLK OK", "I2C...", "UART WAIT", "ADC WAIT",
        "APP WAIT");
    delay_ms(200U);

    /*
     * 剩余外设在系统时钟就绪后统一拉起。
     * 这样 UART/I2C/PWM 的频率配置都按正式工作时钟来生效。
     */
    SYSCFG_DL_PWM_init();
    Board_ShowBootProgress("CLK OK", "I2C OK", "UART...", "ADC WAIT",
        "APP WAIT");
    delay_ms(100U);

    SYSCFG_DL_JY61P_init();
    SYSCFG_DL_JQ8400_init();
    SYSCFG_DL_Exchange_init();
    Board_ShowBootProgress("CLK OK", "I2C OK", "UART OK", "ADC WAIT",
        "APP WAIT");
    delay_ms(100U);

    SYSCFG_DL_GRAY_ADC0_init();
    SYSCFG_DL_GRAY_ADC1_init();
    Board_ShowBootProgress("CLK OK", "I2C OK", "UART OK", "ADC OK",
        "APP WAIT");
    delay_ms(100U);

    Motor_Init();
    Gray_Init();
    Encoder_Init();
    Key_Init();
    JY61P_Init();
    JQ8400_Init();
    Link_Init();

    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
}

void Board_Task(void)
{
}
