#include "board.h"

#include "board_config.h"
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

#define BOARD_BOOT_STEP_DELAY_MS    (80U)
#define BOARD_DEBUG_LED_PORT        (GPIOA)
#define BOARD_DEBUG_LED_IOMUX       (IOMUX_PINCM36)
#define BOARD_DEBUG_LED_PIN         (DL_GPIO_PIN_14)
#define BOARD_FATAL_ERROR_MASK      (BOARD_ERROR_CLOCK)
#define BOARD_SIGNAL_BLINK_TICKS    (120000U)
#define BOARD_ERROR_BLINK_TICKS     (25000U)

static uint32_t g_boardErrors;

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
    if (OLED_HasError() != 0U) {
        Board_ReportError(BOARD_ERROR_OLED_I2C);
        return;
    }
    OLED_ColorTurn(0U);
    OLED_DisplayTurn(0U);
    if (OLED_HasError() != 0U) {
        Board_ReportError(BOARD_ERROR_OLED_I2C);
        return;
    }
    Board_ShowBootLine(0U, clkStatus);
    Board_ShowBootLine(1U, i2cStatus);
    Board_ShowBootLine(2U, uartStatus);
    Board_ShowBootLine(3U, adcStatus);
    Board_ShowBootLine(4U, appStatus);
    OLED_Refresh();
    if (OLED_HasError() != 0U) {
        Board_ReportError(BOARD_ERROR_OLED_I2C);
    }
}

/*
 * 作用：按初始化顺序显示当前正在执行的步骤。
 * 使用场景：某个外设初始化卡死时，OLED 会停在对应 RUN 行。
 */
static void Board_ShowBootStep(const char *done, const char *running,
    const char *waiting1, const char *waiting2)
{
    if (OLED_HasError() != 0U) {
        Board_ReportError(BOARD_ERROR_OLED_I2C);
        return;
    }
    Board_ShowBootProgress("Boot init", done, running, waiting1, waiting2);
    if (OLED_HasError() == 0U) {
        delay_ms(BOARD_BOOT_STEP_DELAY_MS);
    }
}

/*
 * 作用：初始化板载 LED 调试灯。
 * 使用场景：临时把按键/状态反馈映射到灯上，便于不看 OLED 也能确认事件。
 * 说明：1.1ccs 已把灰度 S1 迁走，PA14 固定作为状态灯。
 */
void Board_DebugLedInit(void)
{
#if CAR_ENABLE_PA14_DEBUG_LED
    DL_GPIO_initDigitalOutput(BOARD_DEBUG_LED_IOMUX);
    DL_GPIO_enableOutput(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
    DL_GPIO_clearPins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
}

/*
 * 作用：直接设置板载 LED 调试灯的亮灭状态。
 * 使用场景：做按键、电平和状态机联动时，直接用灯观察当前状态。
 */
void Board_DebugLedSet(uint8_t enabled)
{
#if CAR_ENABLE_PA14_DEBUG_LED
    if (enabled != 0U) {
        DL_GPIO_setPins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
    } else {
        DL_GPIO_clearPins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
    }
#else
    (void)enabled;
#endif
}

/*
 * 作用：翻转板载 LED 调试灯。
 * 使用场景：按键边沿触发测试、状态变化确认。
 */
void Board_DebugLedToggle(void)
{
#if CAR_ENABLE_PA14_DEBUG_LED
    DL_GPIO_togglePins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
}

/*
 * 作用：记录板级错误并点亮调试灯。
 * 使用场景：OLED I2C 超时、系统 PLL 初始化失败等不能沉默的异常。
 */
void Board_ReportError(BoardErrorCode error)
{
    g_boardErrors |= (uint32_t)error;
#if CAR_ENABLE_PA14_DEBUG_LED
    DL_GPIO_setPins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
}

uint32_t Board_GetErrors(void)
{
    return g_boardErrors;
}

uint8_t Board_HasFatalError(void)
{
    return ((g_boardErrors & BOARD_FATAL_ERROR_MASK) != 0U) ? 1U : 0U;
}

void Board_Init(void)
{
    g_boardErrors = BOARD_ERROR_NONE;

    /*
     * 先只初始化电源、GPIO 和 OLED。
     * 这样如果系统时钟卡住，屏幕还能停在启动探针页。
     */
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    Board_DebugLedInit();
    SYSCFG_DL_OLED_init();
    OLED_Init();
    if (OLED_HasError() != 0U) {
        Board_ReportError(BOARD_ERROR_OLED_I2C);
    } else {
        OLED_ColorTurn(0U);
        OLED_DisplayTurn(0U);
        OLED_Clear();
        Board_ShowBootStep("OK Power GPIO", "RUN Clock", "WAIT UART ADC",
            "WAIT Drivers");
    }

    /*
     * 再初始化系统时钟。
     * 这里如果 HFXT / SYSPLL 不稳定，屏幕会明显卡在上一页。
     */
    SYSCFG_DL_SYSCTL_init();
    if (!SYSCFG_DL_SYSCTL_isClockOk()) {
        Board_ReportError(BOARD_ERROR_CLOCK);
        Board_ShowBootProgress("CLK ERR", "PLL timeout", "LED ON",
            "STOP INIT", "");
        return;
    }

    SYSCFG_DL_OLED_init();
    OLED_Init();
    if (OLED_HasError() != 0U) {
        Board_ReportError(BOARD_ERROR_OLED_I2C);
    }
    Board_ShowBootStep("OK Clock OLED", "RUN Stepper", "WAIT UART ADC",
        "WAIT Drivers");

    /*
     * 剩余外设在系统时钟就绪后统一拉起。
     * 这样 UART/I2C/ADC 的频率配置都按正式工作时钟来生效。
     */
    SYSCFG_DL_PWM_init();
    Board_ShowBootStep("OK Stepper GPIO", "RUN UART", "WAIT ADC",
        "WAIT Drivers");

    SYSCFG_DL_JY61P_init();
    SYSCFG_DL_JQ8400_init();
    SYSCFG_DL_Exchange_init();
    Board_ShowBootStep("OK UART x3", "RUN Gray ADC", "WAIT Drivers",
        "WAIT App");

    SYSCFG_DL_GRAY_ADC0_init();
    SYSCFG_DL_GRAY_ADC1_init();
    Board_ShowBootStep("OK ADC", "RUN Motor", "WAIT Gray Enc",
        "WAIT Key UART");
    Motor_Init();
    Board_ShowBootStep("OK Motor", "RUN Gray", "WAIT Key",
        "WAIT Key UART");
    Gray_Init();
    Board_ShowBootStep("OK Gray", "RUN Key", "WAIT UART Wrap",
        "WAIT App");
#if CAR_ENABLE_ENCODER_INPUTS
    Board_ShowBootStep("OK Gray", "RUN Encoder", "WAIT Key",
        "WAIT UART Wrap");
    Encoder_Init();
    Board_ShowBootStep("OK Encoder", "RUN Key", "WAIT UART Wrap",
        "WAIT App");
#endif
    Key_Init();
    Board_ShowBootStep("OK Key", "RUN JY61P", "WAIT JQ Link",
        "WAIT App");
    JY61P_Init();
    Board_ShowBootStep("OK JY61P", "RUN JQ8400", "WAIT Link",
        "WAIT App");
    JQ8400_Init();
    Board_ShowBootStep("OK JQ8400", "RUN Link", "WAIT App",
        "");
    Link_Init();

    Board_ShowBootStep("OK Board", "RUN App", "", "");
}

/*
 * 作用：用板载灯输出系统状态信号。
 * 使用场景：OLED 没接、I2C 卡线或换板测试时，用肉眼判断程序是否还在主循环。
 * 显示规则：PA14 慢闪表示主循环存活；PA14 快闪表示普通错误；PA14 常亮表示致命错误。
 * 说明：只有 CAR_ENABLE_PA14_DEBUG_LED 为 1 时才真正驱动 PA14。
 */
void Board_Task(void)
{
    static uint32_t signalCounter;
    static uint32_t errorCounter;

    if (Board_HasFatalError() != 0U) {
#if CAR_ENABLE_PA14_DEBUG_LED
        DL_GPIO_setPins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
        return;
    }

    if (g_boardErrors != BOARD_ERROR_NONE) {
        ++errorCounter;
        if (errorCounter >= BOARD_ERROR_BLINK_TICKS) {
            errorCounter = 0U;
#if CAR_ENABLE_PA14_DEBUG_LED
            DL_GPIO_togglePins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
        }
    } else {
        errorCounter = 0U;
        ++signalCounter;
        if (signalCounter >= BOARD_SIGNAL_BLINK_TICKS) {
            signalCounter = 0U;
#if CAR_ENABLE_PA14_DEBUG_LED
            DL_GPIO_togglePins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
        }
    }
}
