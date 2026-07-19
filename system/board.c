#include "board.h"

#include "board_config.h"
#include "delay.h"
#include "gray.h"
#include "h7_lcd_display.h"
#include "interrupt.h"
#include "key.h"
#include "link.h"
#include "log_uart.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

#define BOARD_BOOT_STEP_DELAY_MS    (80U)
#define BOARD_DEBUG_LED_PORT        (GPIOA)
#define BOARD_DEBUG_LED_IOMUX       (IOMUX_PINCM36)
#define BOARD_DEBUG_LED_PIN         (DL_GPIO_PIN_14)
#define BOARD_FATAL_ERROR_MASK      (BOARD_ERROR_CLOCK)
/* Board_Task由20ms UI任务调用，50拍约1s翻转一次。 */
#define BOARD_SIGNAL_BLINK_TICKS    (50U)
/* 非致命错误用更快频率提示。 */
#define BOARD_ERROR_BLINK_TICKS     (10U)
#define BOARD_RECOVERY_BLINK_CYCLES (16000000U)
#define BOARD_RECOVERY_POWER_DELAY  (16U)
#define BOARD_RECOVERY_UART_TIMEOUT (100000U)
#define BOARD_RECOVERY_UART_TEXT \
    "RECOVERY SAFE BUILD RUNNING, PA14 BLINK, UART OK\r\n"
#define BOARD_BOOT_MAX_CHARS        (21U)
#define BOARD_BOOT_VISIBLE_LINES    (4U)

static uint32_t g_boardErrors;
static uint32_t g_boardResetCause;

#if CAR_RECOVERY_SAFE_BUILD
static void Board_RecoveryUartInit(void);
static void Board_RecoveryUartSendAll(const char *text);
static uint8_t Board_RecoveryUartSendByte(UART_Regs *uart, uint8_t data);
#endif

#if CAR_ENABLE_LOG_UART
/* 作用：输出上一次复位原因，区分WWDT、CPU锁死、掉电和普通复位。 */
static void Board_LogResetCause(void)
{
    LOG_HEX32("reset cause raw=", g_boardResetCause);
    switch ((DL_SYSCTL_RESET_CAUSE)g_boardResetCause) {
    case DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT0_VIOLATION:
        LOG_LINE("reset cause: WWDT0 SYSRST");
        break;
    case DL_SYSCTL_RESET_CAUSE_SYSRST_CPU_LOCKUP_VIOLATION:
        LOG_LINE("reset cause: CPU LOCKUP");
        break;
    case DL_SYSCTL_RESET_CAUSE_BOR_SUPPLY_FAILURE:
        LOG_LINE("reset cause: BOR SUPPLY");
        break;
    case DL_SYSCTL_RESET_CAUSE_POR_EXTERNAL_NRST:
        LOG_LINE("reset cause: POR NRST");
        break;
    case DL_SYSCTL_RESET_CAUSE_BOOTRST_EXTERNAL_NRST:
        LOG_LINE("reset cause: BOOT NRST");
        break;
    default:
        LOG_LINE("reset cause: other");
        break;
    }
}
#endif

/*
 * 作用：清掉H7 LCD上的上一页启动探针内容。
 * 使用场景：每次刷新启动探针页前调用。
 */
static void Board_ClearBootArea(void)
{
    H7LcdDisplay_Clear();
}

/*
 * 作用：把启动阶段的一行状态写到H7 LCD缓存。
 * 使用场景：排查时钟、I2C、UART、灰度输入、应用层初始化卡在哪一步。
 */
static void Board_ShowBootLine(uint8_t line, const char *text)
{
    char buffer[BOARD_BOOT_MAX_CHARS + 1U];
    uint8_t index;

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

    H7LcdDisplay_ShowLine(line, buffer);
}

/*
 * 作用：在上电阶段缓存并显示当前初始化进度。
 * 使用场景：排查是否卡在时钟初始化、外设初始化或复位边界。
 */
void Board_ShowBootProgress(const char *clkStatus, const char *i2cStatus,
    const char *uartStatus, const char *adcStatus, const char *appStatus)
{
    Board_ClearBootArea();
    Board_ShowBootLine(0U, clkStatus);
    Board_ShowBootLine(1U, i2cStatus);
    Board_ShowBootLine(2U, uartStatus);
    Board_ShowBootLine(3U, adcStatus);
    if ((appStatus != 0) && (appStatus[0] != '\0')) {
        H7LcdDisplay_ShowLine(4U, appStatus);
    }
    H7LcdDisplay_Refresh();
}

/*
 * 作用：按初始化顺序显示当前正在执行的步骤。
 * 使用场景：某个外设初始化卡死时，H7 LCD会停在对应RUN行。
 */
static void Board_ShowBootStep(const char *done, const char *running,
    const char *waiting1, const char *waiting2)
{
    Board_ShowBootProgress(done, running, waiting1, waiting2, "");
    delay_ms(BOARD_BOOT_STEP_DELAY_MS);
}

#if CAR_RECOVERY_SAFE_BUILD
/*
 * 作用：恢复安全模式下只拉起串口打印，不初始化其它外设。
 * 使用场景：XDS110/CCS 下载流程不稳定时，用串口判断程序是否真的烧录并运行。
 * 说明：同时打开日志和视觉两路 UART，便于恢复时观察输出。
 */
static void Board_RecoveryUartInit(void)
{
    DL_UART_Main_reset(LogUart_INST);
    DL_UART_Main_reset(Exchange_INST);

    DL_UART_Main_enablePower(LogUart_INST);
    DL_UART_Main_enablePower(Exchange_INST);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_LogUart_IOMUX_TX, GPIO_LogUart_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_LogUart_IOMUX_RX, GPIO_LogUart_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_Exchange_IOMUX_TX, GPIO_Exchange_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_Exchange_IOMUX_RX, GPIO_Exchange_IOMUX_RX_FUNC);

    SYSCFG_DL_LogUart_init();
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
 * 作用：把恢复心跳同时打印到两路 UART。
 * 使用场景：不知道 USB-TTL 现在接在哪一路 TX 时，两路都能看到同一条消息。
 */
static void Board_RecoveryUartSendAll(const char *text)
{
    while ((text != NULL) && (*text != '\0')) {
        (void)Board_RecoveryUartSendByte(LogUart_INST, (uint8_t)*text);
        (void)Board_RecoveryUartSendByte(Exchange_INST, (uint8_t)*text);
        ++text;
    }
}
#endif

/*
 * 作用：初始化板载 LED 调试灯。
 * 使用场景：临时把按键/状态反馈映射到灯上，便于不看 OLED 也能确认事件。
 * 说明：ccs1.2 已把灰度 S1 迁走，PA14 固定作为状态灯。
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
 * 作用：Board_Init 后半段诊断探针。
 * 使用场景：UART 已初始化后，用 Type-C 日志和 PA14 翻转定位卡在哪个初始化步骤。
 */
static void Board_BootProbe(const char *stage)
{
#if CAR_ENABLE_LOG_UART
    LOG_RAW("[BOOT] ");
    LOG_LINE(stage);
#else
    (void)stage;
#endif
    Board_DebugLedToggle();
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

uint32_t Board_GetResetCause(void)
{
    return g_boardResetCause;
}

uint8_t Board_HasFatalError(void)  //有致命错误返回 1，没有返回 0
{
    return ((g_boardErrors & BOARD_FATAL_ERROR_MASK) != 0U) ? 1U : 0U;
}

uint8_t Board_IsDisplayAvailable(void)
{
    return H7LcdDisplay_IsReady();
}

void Board_Init(void)
{
    /* RSTCAUSE可能被后续启动代码读取，必须在任何外设初始化前先锁存。 */
    g_boardResetCause = (uint32_t)DL_SYSCTL_getResetCause();
    g_boardErrors = BOARD_ERROR_NONE;

#if CAR_RECOVERY_SAFE_BUILD
    /*
     * 恢复安全模式：
     * 只用内部 SYSOSC，关闭 HFXT/PLL，只给 GPIOA/GPIOB 上电并配置 PA14、UART。
     * 不调用 SYSCFG_DL_initPower()，避免整口 reset GPIOA 后影响 SWD 默认状态。
     * 不初始化 OLED/I2C/灰度输入/步进电机，只开三路 UART 打印心跳。
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
     * 先初始化电源和GPIO；H7 LCD内容先写软件缓存，UART0就绪后统一发出。
     */
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    Board_DebugLedInit();
    H7LcdDisplay_Init();
    Board_ShowBootStep("OK Power GPIO", "RUN Clock", "WAIT UART Gray",
        "WAIT Drivers");

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

    Interrupt_Init();

    Board_ShowBootStep("OK Clock", "RUN Stepper", "WAIT UART Gray",
        "WAIT Drivers");

    /*
     * 剩余外设在系统时钟就绪后统一拉起。
     * 这样 UART/I2C/灰度输入的频率配置都按正式工作时钟来生效。
     * 步进 STEP/DIR GPIO 已在 SYSCFG_DL_GPIO_init() 中完成。
     */
    Board_ShowBootStep("OK Stepper GPIO", "RUN UART", "WAIT Gray",
        "WAIT Drivers");

    SYSCFG_DL_Exchange_init();
    SYSCFG_DL_JY61P_init();
#if CAR_ENABLE_LOG_UART
    SYSCFG_DL_LogUart_init();
    LogUart_Init();
    H7LcdDisplay_SetReady(1U);
    H7LcdDisplay_Refresh();
    Board_LogResetCause();
    Board_BootProbe("after log uart init");
    LOG_LINE("board uart: log/exchange ok");
    LOG_U32("clock mclk hz=", CPUCLK_FREQ);
    LOG_U32("clock bus hz=", LogUart_INST_FREQUENCY);
    LOG_U32("step timer clk hz=", STEPPER_TIMER_CLOCK_HZ);
    LOG_U32("step tick hz=", STEPPER_TIMER_TICK_HZ);
    LOG_U32("step load=", STEPPER_TIMER_LOAD_VALUE);
    LOG_U32("gray sample tick hz=", GRAY_SAMPLE_TIMER_TICK_HZ);
    LOG_U32("step ramp ms=", CAR_STEPPER_RAMP_PERIOD_MS);
    LOG_U32("step accel sps/ramp=", CAR_STEPPER_ACCEL_STEP_SPS);
    LOG_U32("step decel sps/ramp=", CAR_STEPPER_DECEL_STEP_SPS);
#endif
#if CAR_ENABLE_LOG_UART
#if CAR_GRAY_INPUT_DIGITAL
    Board_BootProbe("before bootstep gray gpio");
    Board_ShowBootStep("OK UART Log/Ex", "RUN Gray GPIO", "WAIT Drivers",
        "WAIT App");
    Board_BootProbe("after bootstep gray gpio");
#else
    Board_BootProbe("before bootstep gray adc");
    Board_ShowBootStep("OK UART Log/Ex", "RUN Gray ADC", "WAIT Drivers",
        "WAIT App");
    Board_BootProbe("after bootstep gray adc");
#endif
#else
#if CAR_GRAY_INPUT_DIGITAL
    Board_ShowBootStep("OK UART Ex", "RUN Gray GPIO", "WAIT Drivers",
        "WAIT App");
#else
    Board_ShowBootStep("OK UART Ex", "RUN Gray ADC", "WAIT Drivers",
        "WAIT App");
#endif
#endif

#if (CAR_GRAY_INPUT_DIGITAL == 0U)
    Board_BootProbe("before gray adc init");
    SYSCFG_DL_GRAY_ADC0_init();
    SYSCFG_DL_GRAY_ADC1_init();
    Board_BootProbe("after gray adc init");
    Board_ShowBootStep("OK ADC", "RUN Stepper TIM", "WAIT Motor",
        "WAIT Gray");
#else
    Board_ShowBootStep("OK Gray GPIO", "RUN Stepper TIM", "WAIT Motor",
        "WAIT Gray");
#endif
    Board_BootProbe("before stepper timer init");
    SYSCFG_DL_STEPPER_TIMER_init();
    SYSCFG_DL_GRAY_SAMPLE_TIMER_init();
    Board_BootProbe("after stepper timer init");
    Board_ShowBootStep("OK Stepper TIM", "RUN Motor", "WAIT Gray",
        "WAIT Key UART");
    Board_BootProbe("before motor init");
    Motor_Init();
    Board_BootProbe("after motor init");
    Board_ShowBootStep("OK Motor", "RUN Gray", "WAIT Key",
        "WAIT Key UART");
    Board_BootProbe("before gray init");
    Gray_Init();
    Board_BootProbe("after gray init");
    Board_ShowBootStep("OK Gray", "RUN Key", "WAIT UART Wrap",
        "WAIT App");
    Board_BootProbe("before key init");
    Key_Init();
    Board_BootProbe("after key init");
    Board_ShowBootStep("OK Key", "RUN Link", "WAIT App", "");
    Board_BootProbe("before link init");
    Link_Init();
    Board_BootProbe("after link init");

    Board_BootProbe("before board done bootstep");
    Board_ShowBootStep("OK Board", "RUN App", "", "");
    Board_BootProbe("after board done bootstep");
#endif
}

/*
 * 作用：用板载灯输出系统状态信号。
 * 使用场景：H7 LCD未连接或换板测试时，用肉眼判断程序是否还在主循环。
 * 显示规则：PA14 慢闪表示主循环存活；PA14 快闪表示普通错误；PA14 常亮表示致命错误。
 * 说明：只有 CAR_ENABLE_PA14_DEBUG_LED 为 1 时才真正驱动 PA14。
 */
uint8_t Board_Task(void)
{
    static uint32_t signalCounter;
    static uint32_t errorCounter;

#if CAR_RECOVERY_SAFE_BUILD
    delay_cycles(BOARD_RECOVERY_BLINK_CYCLES);
#if CAR_ENABLE_PA14_DEBUG_LED
    DL_GPIO_togglePins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
    Board_RecoveryUartSendAll(BOARD_RECOVERY_UART_TEXT);
    return 0U;
#endif

    if (Board_HasFatalError() != 0U) {
#if CAR_ENABLE_PA14_DEBUG_LED
        DL_GPIO_setPins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
        return 0U;
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
    return 0U;
}
