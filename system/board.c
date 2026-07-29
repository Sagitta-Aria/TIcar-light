/*
 * MSPM0G3507板级初始化与维护：按安全顺序配置时钟、UART、传感器、显示和电机。
 * 负责锁存复位原因、板级错误与板载灯心跳；不会创建RTOS任务。
 * generated/ti_msp_dl_config由SysConfig拥有，本文件只在手写扩展点配置运行时行为。
 */
#include "board.h"

#include "board_config.h"
#include "car_display.h"
#include "delay.h"
#if CAR_LIBRARY_GRAY_INPUT_ENABLED
#include "gray.h"
#endif
#if CAR_PROFILE_IS_GMR
#include "h7_control_uart.h"
#endif
#include "interrupt.h"
#if CAR_LIBRARY_IMU660RX_ENABLED
#include "imu660rx.h"
#endif
#include "key.h"
#if CAR_PROFILE_IS_FULL
#include "link.h"
#endif
#include "log_uart.h"
#include "motor.h"
#include "pin_map.h"
#include "resource_config.h"
#include "ti_msp_dl_config.h"

#define BOARD_BOOT_STEP_DELAY_MS    (80U)
#define BOARD_DEBUG_LED_PORT        PIN_DEBUG_LED_PORT
#define BOARD_DEBUG_LED_IOMUX       PIN_DEBUG_LED_IOMUX
#define BOARD_DEBUG_LED_PIN         PIN_DEBUG_LED
#define BOARD_FATAL_ERROR_MASK      (BOARD_ERROR_CLOCK)
/* Board_Task由20ms UI任务调用，50拍约1s翻转一次。 */
#define BOARD_SIGNAL_BLINK_TICKS    (50U)
/* 非致命错误用更快频率提示。 */
#define BOARD_ERROR_BLINK_TICKS     (10U)
#define BOARD_RECOVERY_BLINK_CYCLES (16000000U)
#define BOARD_RECOVERY_POWER_DELAY  (16U)
#define BOARD_RECOVERY_UART_TIMEOUT (100000U)
#define BOARD_RECOVERY_UART_TEXT \
    "RECOVERY SAFE BUILD RUNNING, DEBUG LED BLINK, UART OK\r\n"
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
 * 作用：清掉所有已启用显示后端的上一页启动探针内容。
 * 使用场景：每次刷新启动探针页前调用。
 */
static void Board_ClearBootArea(void)
{
    CarDisplay_Clear();
}

/*
 * 作用：把启动阶段的一行状态写到统一显示出口。
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

    CarDisplay_ShowLine(line, buffer);
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
        CarDisplay_ShowLine(4U, appStatus);
    }
    CarDisplay_Refresh();
}

/*
 * 作用：按初始化顺序显示当前正在执行的步骤。
 * 使用场景：某个外设初始化卡死时，已启用屏幕会停在对应RUN行。
 */
static void Board_ShowBootStep(const char *done, const char *running,
    const char *waiting1, const char *waiting2)
{
#if CAR_LIBRARY_DISPLAY_ENABLED
    Board_ShowBootProgress(done, running, waiting1, waiting2, "");
    delay_ms(BOARD_BOOT_STEP_DELAY_MS);
#else
    (void)done;
    (void)running;
    (void)waiting1;
    (void)waiting2;
#endif
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
 * 说明：实际引脚由CAR_LIBRARY_BOARD_PROFILE决定。
 */
void Board_DebugLedInit(void)
{
#if CAR_ENABLE_DEBUG_LED
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
#if CAR_ENABLE_DEBUG_LED
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
#if CAR_ENABLE_DEBUG_LED
    DL_GPIO_togglePins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
}

/*
 * 作用：Board_Init 后半段诊断探针。
 * 使用场景：UART已初始化后，用Type-C日志和板载灯翻转定位初始化步骤。
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
#if CAR_ENABLE_DEBUG_LED
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
    return CarDisplay_IsAvailable();
}

void Board_Init(void)
{
#if CAR_LIBRARY_IMU660RX_ENABLED
    IMU660RXStatus imu660rxStatus;
#endif

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
     * 先初始化电源和GPIO；H7 LCD先写缓存，OLED等待正式时钟后初始化。
     */
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    Board_DebugLedInit();
    CarDisplay_Init();
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

#if CAR_LIBRARY_LOCAL_OLED_ENABLED
    /* OLED依赖正式系统时钟；失败只记录非致命I2C错误，车辆控制仍可运行。 */
    if (CarDisplay_InitLocalOled() == 0U) {
        Board_ReportError(BOARD_ERROR_OLED_I2C);
    }
#endif

    Interrupt_Init();

#if CAR_PROFILE_IS_GMR
    Board_ShowBootStep("OK Clock", "RUN UART0", "WAIT Motor",
        "WAIT Key");
#else
    Board_ShowBootStep("OK Clock", "RUN Stepper", "WAIT UART Gray",
        "WAIT Drivers");
#endif

    /*
     * 剩余外设在系统时钟就绪后统一拉起。
     * 这样 UART/I2C/灰度输入的频率配置都按正式工作时钟来生效。
     * 步进 STEP/DIR GPIO 已在 SYSCFG_DL_GPIO_init() 中完成。
     */
    Board_ShowBootStep("OK Stepper GPIO", "RUN UART", "WAIT Gray",
        "WAIT Drivers");

#if CAR_PROFILE_IS_FULL || CAR_M0_ATTITUDE_UART_REQUIRED
    SYSCFG_DL_Exchange_init();
#endif
#if CAR_PROFILE_IS_GMR && CAR_H7_UART_REQUIRED
    H7ControlUart_Init();
#endif
#if CAR_JY61P_ENABLED
    SYSCFG_DL_JY61P_init();
#endif
#if CAR_UART0_REQUIRED
    SYSCFG_DL_LogUart_init();
    LogUart_Init();
#endif
#if CAR_LIBRARY_H7_LCD_ENABLED
    CarDisplay_SetH7Ready(1U);
    CarDisplay_Refresh();
#endif
#if CAR_ENABLE_LOG_UART
    Board_LogResetCause();
    Board_BootProbe("after log uart init");
    LOG_LINE("board uart: log/exchange ok");
    LOG_U32("clock mclk hz=", CPUCLK_FREQ);
    LOG_U32("clock bus hz=", LogUart_INST_FREQUENCY);
#if CAR_PROFILE_IS_FULL
    LOG_U32("step timer clk hz=", STEPPER_TIMER_CLOCK_HZ);
    LOG_U32("step tick hz=", STEPPER_TIMER_TICK_HZ);
    LOG_U32("step load=", STEPPER_TIMER_LOAD_VALUE);
    LOG_U32("gray sample tick hz=", GRAY_SAMPLE_TIMER_TICK_HZ);
    LOG_U32("step ramp ms=", CAR_STEPPER_RAMP_PERIOD_MS);
    LOG_U32("step accel sps/ramp=", CAR_STEPPER_ACCEL_STEP_SPS);
    LOG_U32("step decel sps/ramp=", CAR_STEPPER_DECEL_STEP_SPS);
#else
    LOG_LINE("board mode: gmr minimal");
#endif
#endif
#if CAR_LIBRARY_IMU660RX_ENABLED
    Board_BootProbe("before imu660rx init");
    Board_ShowBootStep("OK UART", "RUN IMU660RX", "WAIT Drivers",
        "WAIT App");
    imu660rxStatus = IMU660RX_Init();
    if (imu660rxStatus != IMU660RX_STATUS_OK) {
        Board_ReportError(BOARD_ERROR_IMU660RX);
    }
#if CAR_ENABLE_LOG_UART
    LOG_U32("imu660rx status=", (uint32_t)imu660rxStatus);
    LOG_U32("imu660rx model=", (uint32_t)IMU660RX_GetModel());
#endif
    Board_BootProbe("after imu660rx init");
#endif
#if CAR_PROFILE_IS_GMR
    Board_BootProbe("before bootstep gmr motor");
    Board_ShowBootStep("OK UART0", "RUN Motor", "WAIT Key",
        "WAIT App");
    Board_BootProbe("after bootstep gmr motor");
#else
#if CAR_UART0_REQUIRED
#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
    Board_BootProbe("before bootstep gray gpio");
    Board_ShowBootStep("OK UART0/Ex", "RUN Gray GPIO", "WAIT Drivers",
        "WAIT App");
    Board_BootProbe("after bootstep gray gpio");
#else
    Board_BootProbe("before bootstep gray adc");
    Board_ShowBootStep("OK UART0/Ex", "RUN Gray ADC", "WAIT Drivers",
        "WAIT App");
    Board_BootProbe("after bootstep gray adc");
#endif
#else
#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
    Board_ShowBootStep("OK UART Ex", "RUN Gray GPIO", "WAIT Drivers",
        "WAIT App");
#else
    Board_ShowBootStep("OK UART Ex", "RUN Gray ADC", "WAIT Drivers",
        "WAIT App");
#endif
#endif

#if !CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
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
    Board_BootProbe("after stepper timer init");
    Board_ShowBootStep("OK Stepper TIM", "RUN Motor", "WAIT Gray",
        "WAIT Key UART");
#endif
#if CAR_LIBRARY_LINE_FOLLOW_ENABLED
    Board_BootProbe("before gray sample timer init");
    SYSCFG_DL_GRAY_SAMPLE_TIMER_init();
    Board_BootProbe("after gray sample timer init");
#endif
    Board_BootProbe("before motor init");
    Motor_Init();
    Board_BootProbe("after motor init");
#if CAR_LIBRARY_GRAY_INPUT_ENABLED
    Board_ShowBootStep("OK Motor", "RUN Gray", "WAIT Key",
        "WAIT App");
    Board_BootProbe("before gray init");
    Gray_Init();
    Board_BootProbe("after gray init");
#if CAR_PROFILE_IS_GMR
    Board_ShowBootStep("OK Gray", "RUN Key", "WAIT App", "");
#else
    Board_ShowBootStep("OK Gray", "RUN Key", "WAIT UART Wrap",
        "WAIT App");
#endif
#elif CAR_PROFILE_IS_GMR
    Board_ShowBootStep("OK Motor", "RUN Key", "WAIT App", "");
#else
    Board_ShowBootStep("OK Motor", "RUN Key", "WAIT UART Wrap",
        "WAIT App");
#endif
    Board_BootProbe("before key init");
    Key_Init();
    Board_BootProbe("after key init");
#if CAR_PROFILE_IS_GMR
    Board_ShowBootStep("OK Key", "RUN App", "", "");
#else
    Board_ShowBootStep("OK Key", "RUN Link", "WAIT App", "");
    Board_BootProbe("before link init");
    Link_Init();
    Board_BootProbe("after link init");
#endif

    Board_BootProbe("before board done bootstep");
    Board_ShowBootStep("OK Board", "RUN App", "", "");
    Board_BootProbe("after board done bootstep");
#endif
}

/*
 * 作用：用板载灯输出系统状态信号。
 * 使用场景：显示屏未连接或换板测试时，用肉眼判断程序是否还在主循环。
 * 显示规则：慢闪表示主循环存活，快闪表示普通错误，常亮表示致命错误。
 * 说明：只有CAR_ENABLE_DEBUG_LED为1时才驱动当前板型的状态灯。
 */
uint8_t Board_Task(void)
{
    static uint32_t signalCounter;
    static uint32_t errorCounter;

#if CAR_RECOVERY_SAFE_BUILD
    delay_cycles(BOARD_RECOVERY_BLINK_CYCLES);
#if CAR_ENABLE_DEBUG_LED
    DL_GPIO_togglePins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
    Board_RecoveryUartSendAll(BOARD_RECOVERY_UART_TEXT);
    return 0U;
#endif

    if (Board_HasFatalError() != 0U) {
#if CAR_ENABLE_DEBUG_LED
        DL_GPIO_setPins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
        return 0U;
    }

    if (g_boardErrors != BOARD_ERROR_NONE) {
        ++errorCounter;
        if (errorCounter >= BOARD_ERROR_BLINK_TICKS) {
            errorCounter = 0U;
#if CAR_ENABLE_DEBUG_LED
            DL_GPIO_togglePins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
        }
    } else {
        errorCounter = 0U;
        ++signalCounter;
        if (signalCounter >= BOARD_SIGNAL_BLINK_TICKS) {
            signalCounter = 0U;
#if CAR_ENABLE_DEBUG_LED
            DL_GPIO_togglePins(BOARD_DEBUG_LED_PORT, BOARD_DEBUG_LED_PIN);
#endif
        }
    }
    return 0U;
}
