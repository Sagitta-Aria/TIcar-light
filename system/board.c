#include "board.h"

#include "board_config.h"
#include "delay.h"
#include "encoder.h"
#include "gray.h"
#include "jy61p.h"
#include "key.h"
#include "link.h"
#include "log_uart.h"
#include "motor.h"
#include "oled.h"
#include "ti_msp_dl_config.h"

#define BOARD_BOOT_STEP_DELAY_MS    (80U)
#define BOARD_DEBUG_LED_PORT        (GPIOA)
#define BOARD_DEBUG_LED_IOMUX       (IOMUX_PINCM36)
#define BOARD_DEBUG_LED_PIN         (DL_GPIO_PIN_14)
#define BOARD_FATAL_ERROR_MASK      (BOARD_ERROR_CLOCK)
/* 正常模式下 App_Task 每轮约 10ms，因此 50 拍约 0.5s 翻转一次。 */
#define BOARD_SIGNAL_BLINK_TICKS    (50U)
/* 非致命错误用更快频率提示，例如 OLED/I2C 超时。 */
#define BOARD_ERROR_BLINK_TICKS     (10U)
#define BOARD_RECOVERY_BLINK_CYCLES (16000000U)
#define BOARD_RECOVERY_POWER_DELAY  (16U)
#define BOARD_RECOVERY_UART_TIMEOUT (100000U)
#define BOARD_RECOVERY_UART_TEXT \
    "RECOVERY SAFE BUILD RUNNING, PA14 BLINK, UART OK\r\n"
#define BOARD_BOOT_FONT_SIZE        (12U)
#define BOARD_BOOT_MAX_CHARS        (21U)
#define BOARD_BOOT_VISIBLE_LINES    (4U)
#define BOARD_BOOT_START_Y          (16U)
#define BOARD_BOOT_LINE_STEP        (12U)

static uint32_t g_boardErrors;

#if CAR_RECOVERY_SAFE_BUILD
static void Board_RecoveryUartInit(void);
static void Board_RecoveryUartSendAll(const char *text);
static uint8_t Board_RecoveryUartSendByte(UART_Regs *uart, uint8_t data);
#endif

/*
 * 作用：错误码变化时打一行日志。
 * 使用场景：OLED/I2C 超时、时钟错误等需要同时从 PA14 和串口确认的场景。
 */
static void Board_LogErrorChange(uint32_t errors)
{
#if CAR_ENABLE_LOG_UART
    if (errors == BOARD_ERROR_NONE) {
        return;
    }

    LOG_RAW("board error mask=");
    LogUart_SendHex32(errors);
    if ((errors & BOARD_ERROR_OLED_I2C) != 0U) {
        LOG_RAW(" OLED_I2C");
    }
    if ((errors & BOARD_ERROR_CLOCK) != 0U) {
        LOG_RAW(" CLOCK");
    }
    LOG_LINE("");
#else
    (void)errors;
#endif
}

/*
 * 作用：清掉 OLED 启动探针内容，尤其避免顶部黄色区域残留标题。
 * 使用场景：每次刷新启动探针页前调用。
 */
static void Board_ClearBootArea(void)
{
    char blank[BOARD_BOOT_MAX_CHARS + 1U];
    uint8_t i;

    for (i = 0U; i < BOARD_BOOT_MAX_CHARS; ++i) {
        blank[i] = ' ';
    }
    blank[BOARD_BOOT_MAX_CHARS] = '\0';

    for (i = 0U; i < 5U; ++i) {
        OLED_ShowLine(i, blank, BOARD_BOOT_FONT_SIZE);
    }
}

/*
 * 作用：把启动阶段的一行状态写到 OLED 下半区。
 * 使用场景：排查时钟、I2C、UART、ADC、应用层初始化卡在哪一步。
 * 说明：从 y=16 开始显示，避开双色 OLED 顶部黄色区域。
 */
static void Board_ShowBootLine(uint8_t line, const char *text)
{
    char buffer[BOARD_BOOT_MAX_CHARS + 1U];
    uint8_t index;
    uint8_t y;

    if (line >= BOARD_BOOT_VISIBLE_LINES) {
        return;
    }

    for (index = 0U; index < BOARD_BOOT_MAX_CHARS; ++index) {
        buffer[index] = ' ';
    }
    buffer[BOARD_BOOT_MAX_CHARS] = '\0';

    if (text != NULL) {
        for (index = 0U; (index < BOARD_BOOT_MAX_CHARS) &&
            (text[index] != '\0'); ++index) {
            buffer[index] = text[index];
        }
    }

    y = (uint8_t)(BOARD_BOOT_START_Y + (line * BOARD_BOOT_LINE_STEP));
    OLED_ShowString(0U, y, (u8 *)buffer, BOARD_BOOT_FONT_SIZE);
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
    Board_ClearBootArea();
    Board_ShowBootLine(0U, clkStatus);
    Board_ShowBootLine(1U, i2cStatus);
    Board_ShowBootLine(2U, uartStatus);
    Board_ShowBootLine(3U, adcStatus);
    (void)appStatus;
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
    Board_ShowBootProgress(done, running, waiting1, waiting2, "");
    if (OLED_HasError() == 0U) {
        delay_ms(BOARD_BOOT_STEP_DELAY_MS);
    }
}

#if CAR_RECOVERY_SAFE_BUILD
/*
 * 作用：恢复安全模式下只拉起串口打印，不初始化其它外设。
 * 使用场景：XDS110/CCS 下载流程不稳定时，用串口判断程序是否真的烧录并运行。
 * 说明：同时打开日志、姿态、视觉三路 UART，便于恢复时观察任意一路。
 */
static void Board_RecoveryUartInit(void)
{
    DL_UART_Main_reset(LogUart_INST);
    DL_UART_Main_reset(JY61P_INST);
    DL_UART_Main_reset(Exchange_INST);

    DL_UART_Main_enablePower(LogUart_INST);
    DL_UART_Main_enablePower(JY61P_INST);
    DL_UART_Main_enablePower(Exchange_INST);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_LogUart_IOMUX_TX, GPIO_LogUart_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_LogUart_IOMUX_RX, GPIO_LogUart_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_JY61P_IOMUX_TX, GPIO_JY61P_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_JY61P_IOMUX_RX, GPIO_JY61P_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_Exchange_IOMUX_TX, GPIO_Exchange_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_Exchange_IOMUX_RX, GPIO_Exchange_IOMUX_RX_FUNC);

    SYSCFG_DL_LogUart_init();
    SYSCFG_DL_JY61P_init();
    SYSCFG_DL_Exchange_init();
}

/*
 * 作用：向单个 UART 发送 1 字节，等待带超时。
 * 使用场景：恢复模式串口心跳。
 * 说明：即使串口外设异常，也不能因为打印把主循环卡死。
 */
static uint8_t Board_RecoveryUartSendByte(UART_Regs *uart, uint8_t data)
{
    uint32_t timeout = BOARD_RECOVERY_UART_TIMEOUT;

    while (timeout > 0U) {
        if (DL_UART_Main_transmitDataCheck(uart, data)) {
            return 1U;
        }
        --timeout;
    }
    return 0U;
}

/*
 * 作用：把恢复心跳同时打印到三路 UART。
 * 使用场景：不知道 USB-TTL 现在接在哪一路 TX 时，三路都能看到同一条消息。
 */
static void Board_RecoveryUartSendAll(const char *text)
{
    while ((text != NULL) && (*text != '\0')) {
        (void)Board_RecoveryUartSendByte(LogUart_INST, (uint8_t)*text);
        (void)Board_RecoveryUartSendByte(JY61P_INST, (uint8_t)*text);
        (void)Board_RecoveryUartSendByte(Exchange_INST, (uint8_t)*text);
        ++text;
    }
}
#endif

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

#if CAR_RECOVERY_SAFE_BUILD
    /*
     * 恢复安全模式：
     * 只用内部 SYSOSC，关闭 HFXT/PLL，只给 GPIOA/GPIOB 上电并配置 PA14、UART。
     * 不调用 SYSCFG_DL_initPower()，避免整口 reset GPIOA 后影响 SWD 默认状态。
     * 不初始化 OLED/I2C/ADC/步进电机，只开三路 UART 打印心跳。
     */
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();

    DL_GPIO_enablePower(BOARD_DEBUG_LED_PORT);
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(BOARD_RECOVERY_POWER_DELAY);
    Board_DebugLedInit();
    Board_RecoveryUartInit();
    Board_RecoveryUartSendAll(BOARD_RECOVERY_UART_TEXT);
    return;
#else
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
     * 步进 STEP/DIR GPIO 已在 SYSCFG_DL_GPIO_init() 中完成。
     */
    Board_ShowBootStep("OK Stepper GPIO", "RUN UART", "WAIT ADC",
        "WAIT Drivers");

    SYSCFG_DL_JY61P_init();
    SYSCFG_DL_Exchange_init();
#if CAR_ENABLE_LOG_UART
    SYSCFG_DL_LogUart_init();
    LogUart_Init();
    LOG_LINE("board uart: log/jy61p/exchange ok");
#endif
#if CAR_ENABLE_LOG_UART
    Board_ShowBootStep("OK UART Log/JY/Ex", "RUN Gray ADC", "WAIT Drivers",
        "WAIT App");
#else
    Board_ShowBootStep("OK UART JY/Ex", "RUN Gray ADC", "WAIT Drivers",
        "WAIT App");
#endif

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
    Board_ShowBootStep("OK Key", "RUN JY61P", "WAIT Link",
        "WAIT App");
    JY61P_Init();
    Board_ShowBootStep("OK JY61P", "RUN Link", "WAIT App", "");
    Link_Init();

    Board_ShowBootStep("OK Board", "RUN App", "", "");
#endif
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
    static uint32_t lastLoggedErrors;

#if CAR_RECOVERY_SAFE_BUILD
    delay_cycles(BOARD_RECOVERY_BLINK_CYCLES);
#if CAR_ENABLE_PA14_DEBUG_LED
    DL_GPIO_togglePins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
    Board_RecoveryUartSendAll(BOARD_RECOVERY_UART_TEXT);
    return;
#endif

    if (Board_HasFatalError() != 0U) {
        if (lastLoggedErrors != g_boardErrors) {
            lastLoggedErrors = g_boardErrors;
            Board_LogErrorChange(g_boardErrors);
        }
#if CAR_ENABLE_PA14_DEBUG_LED
        DL_GPIO_setPins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
        return;
    }

    if (g_boardErrors != BOARD_ERROR_NONE) {
        if (lastLoggedErrors != g_boardErrors) {
            lastLoggedErrors = g_boardErrors;
            Board_LogErrorChange(g_boardErrors);
        }
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
