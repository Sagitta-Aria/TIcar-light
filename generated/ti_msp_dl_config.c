#include "ti_msp_dl_config.h"

SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_OLED_init();
    SYSCFG_DL_LogUart_init();
    SYSCFG_DL_JY61P_init();
    SYSCFG_DL_Exchange_init();
    SYSCFG_DL_GRAY_ADC0_init();
    SYSCFG_DL_GRAY_ADC1_init();
}

SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    return true;
}

SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    return true;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_I2C_reset(OLED_INST);
    DL_UART_Main_reset(LogUart_INST);
    DL_UART_Main_reset(JY61P_INST);
    DL_UART_Main_reset(Exchange_INST);
    DL_ADC12_reset(GRAY_ADC0_INST);
    DL_ADC12_reset(GRAY_ADC1_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_I2C_enablePower(OLED_INST);
    DL_UART_Main_enablePower(LogUart_INST);
    DL_UART_Main_enablePower(JY61P_INST);
    DL_UART_Main_enablePower(Exchange_INST);
    DL_ADC12_enablePower(GRAY_ADC0_INST);
    DL_ADC12_enablePower(GRAY_ADC1_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);

    DL_GPIO_initDigitalOutput(STEPPER_CHASSIS_LEFT_STEP_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_CHASSIS_LEFT_DIR_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_CHASSIS_RIGHT_STEP_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_CHASSIS_RIGHT_DIR_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_GIMBAL_1_STEP_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_GIMBAL_1_DIR_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_GIMBAL_2_STEP_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_GIMBAL_2_DIR_IOMUX);

    /*
     * OLED I2C0 使用 PA0/PA1。
     * HIZ1 使高电平为释放状态，配合上拉形成开漏 I2C 总线。
     * 内部弱上拉只做兜底，实车仍建议在 SDA/SCL 上加 4.7k~10k 外部上拉。
     */
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_OLED_IOMUX_SDA, GPIO_OLED_IOMUX_SDA_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_OLED_IOMUX_SCL, GPIO_OLED_IOMUX_SCL_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SCL);

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

    DL_GPIO_initPeripheralAnalogFunction(GRAY_S1_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GRAY_S2_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GRAY_S3_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GRAY_S4_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GRAY_S5_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GRAY_S6_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GRAY_S7_IOMUX);

    DL_GPIO_initDigitalInputFeatures(KEY_1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY_2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(STEPPER_CHASSIS_LEFT_STEP_PORT,
        STEPPER_CHASSIS_LEFT_STEP_PIN);
    DL_GPIO_clearPins(STEPPER_CHASSIS_LEFT_DIR_PORT,
        STEPPER_CHASSIS_LEFT_DIR_PIN);
    DL_GPIO_clearPins(STEPPER_CHASSIS_RIGHT_STEP_PORT,
        STEPPER_CHASSIS_RIGHT_STEP_PIN | STEPPER_CHASSIS_RIGHT_DIR_PIN);
    DL_GPIO_clearPins(STEPPER_GIMBAL_1_STEP_PORT,
        STEPPER_GIMBAL_1_STEP_PIN | STEPPER_GIMBAL_1_DIR_PIN);
    DL_GPIO_clearPins(STEPPER_GIMBAL_2_STEP_PORT,
        STEPPER_GIMBAL_2_STEP_PIN);
    DL_GPIO_clearPins(STEPPER_GIMBAL_2_DIR_PORT,
        STEPPER_GIMBAL_2_DIR_PIN);

    DL_GPIO_enableOutput(STEPPER_CHASSIS_LEFT_STEP_PORT,
        STEPPER_CHASSIS_LEFT_STEP_PIN);
    DL_GPIO_enableOutput(STEPPER_CHASSIS_LEFT_DIR_PORT,
        STEPPER_CHASSIS_LEFT_DIR_PIN);
    DL_GPIO_enableOutput(STEPPER_CHASSIS_RIGHT_STEP_PORT,
        STEPPER_CHASSIS_RIGHT_STEP_PIN | STEPPER_CHASSIS_RIGHT_DIR_PIN);
    DL_GPIO_enableOutput(STEPPER_GIMBAL_1_STEP_PORT,
        STEPPER_GIMBAL_1_STEP_PIN | STEPPER_GIMBAL_1_DIR_PIN);
    DL_GPIO_enableOutput(STEPPER_GIMBAL_2_STEP_PORT,
        STEPPER_GIMBAL_2_STEP_PIN);
    DL_GPIO_enableOutput(STEPPER_GIMBAL_2_DIR_PORT,
        STEPPER_GIMBAL_2_DIR_PIN);

    DL_GPIO_setLowerPinsPolarity(KEY_PORT,
        DL_GPIO_PIN_9_EDGE_RISE | DL_GPIO_PIN_8_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(KEY_PORT, KEY_1_PIN | KEY_2_PIN);
    DL_GPIO_enableInterrupt(KEY_PORT, KEY_1_PIN | KEY_2_PIN);

}

/*
 * 当前优先保证 J-Link 可重新接管，默认不启用 HFXT/SYSPLL。
 * 如果后续确认板上 32-48MHz 外部晶振稳定，再恢复 PLL 高速时钟。
 */
#if SYSCFG_DL_ENABLE_HFXT_PLL
static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
    .rDivClk2x = 1,
    .rDivClk1 = 0,
    .rDivClk0 = 0,
    .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
    .enableCLK1 = DL_SYSCTL_SYSPLL_CLK1_ENABLE,
    .enableCLK0 = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
    .sysPLLMCLK = DL_SYSCTL_SYSPLL_MCLK_CLK0,
    .sysPLLRef = DL_SYSCTL_SYSPLL_REF_HFCLK,
    .qDiv = 3,
    .pDiv = DL_SYSCTL_SYSPLL_PDIV_1
};
#endif

#define SYSCFG_DL_SYSCTL_PLL_RETRY_MAX       (3U)
#define SYSCFG_DL_SYSCTL_PLL_OFF_TIMEOUT     (100000U)
#define SYSCFG_DL_SYSCTL_PLL_GOOD_TIMEOUT    (300000U)
#define SYSCFG_DL_SYSCTL_HSCLK_TIMEOUT       (100000U)

static volatile bool g_sysctlClockOk;

bool SYSCFG_DL_SYSCTL_isClockOk(void)
{
    return g_sysctlClockOk;
}

/*
 * 作用：等待时钟状态位达到期望值。
 * 使用场景：PLL 关闭、PLL 锁定、HSCLK 切换确认。
 * 说明：所有等待都带超时，避免晶振/PLL 异常时死等。
 */
static bool SYSCFG_DL_SYSCTL_waitClockStatus(uint32_t mask, uint32_t expected,
    uint32_t timeout)
{
    while (timeout > 0U) {
        if ((DL_SYSCTL_getClockStatus() & mask) == expected) {
            return true;
        }
        --timeout;
    }
    return false;
}

/*
 * 作用：按 TI DriverLib 的 PLL 配置流程写寄存器，但把等待改成超时等待。
 * 使用场景：替代 DL_SYSCTL_configSYSPLL()，因为官方函数内部会一直等 PLL good。
 */
static bool SYSCFG_DL_SYSCTL_configSYSPLLTimeout(
    const DL_SYSCTL_SYSPLLConfig *config)
{
    uint32_t ctlTemp;

    DL_SYSCTL_disableSYSPLL();
    if (!SYSCFG_DL_SYSCTL_waitClockStatus(DL_SYSCTL_CLK_STATUS_SYSPLL_OFF,
        DL_SYSCTL_CLK_STATUS_SYSPLL_OFF,
        SYSCFG_DL_SYSCTL_PLL_OFF_TIMEOUT)) {
        return false;
    }

    DL_Common_updateReg(&SYSCTL->SOCLOCK.SYSPLLCFG0,
        ((uint32_t)config->sysPLLRef), SYSCTL_SYSPLLCFG0_SYSPLLREF_MASK);
    DL_Common_updateReg(&SYSCTL->SOCLOCK.SYSPLLCFG1,
        ((uint32_t)config->pDiv), SYSCTL_SYSPLLCFG1_PDIV_MASK);

    ctlTemp = DL_CORE_getInstructionConfig();
    DL_CORE_configInstruction(DL_CORE_PREFETCH_ENABLED,
        DL_CORE_CACHE_DISABLED, DL_CORE_LITERAL_CACHE_ENABLED);
    SYSCTL->SOCLOCK.SYSPLLPARAM0 =
        *(volatile uint32_t *)((uint32_t)config->inputFreq);
    SYSCTL->SOCLOCK.SYSPLLPARAM1 =
        *(volatile uint32_t *)((uint32_t)config->inputFreq + (uint32_t)0x4);
    CPUSS->CTL = ctlTemp;

    DL_Common_updateReg(&SYSCTL->SOCLOCK.SYSPLLCFG1,
        ((config->qDiv << SYSCTL_SYSPLLCFG1_QDIV_OFS) &
            SYSCTL_SYSPLLCFG1_QDIV_MASK),
        SYSCTL_SYSPLLCFG1_QDIV_MASK);
    DL_Common_updateReg(&SYSCTL->SOCLOCK.SYSPLLCFG0,
        (((config->rDivClk2x << SYSCTL_SYSPLLCFG0_RDIVCLK2X_OFS) &
             SYSCTL_SYSPLLCFG0_RDIVCLK2X_MASK) |
            ((config->rDivClk1 << SYSCTL_SYSPLLCFG0_RDIVCLK1_OFS) &
                SYSCTL_SYSPLLCFG0_RDIVCLK1_MASK) |
            ((config->rDivClk0 << SYSCTL_SYSPLLCFG0_RDIVCLK0_OFS) &
                SYSCTL_SYSPLLCFG0_RDIVCLK0_MASK) |
            config->enableCLK2x | config->enableCLK1 | config->enableCLK0 |
            (uint32_t)config->sysPLLMCLK),
        (SYSCTL_SYSPLLCFG0_RDIVCLK2X_MASK |
            SYSCTL_SYSPLLCFG0_RDIVCLK1_MASK |
            SYSCTL_SYSPLLCFG0_RDIVCLK0_MASK |
            SYSCTL_SYSPLLCFG0_ENABLECLK2X_MASK |
            SYSCTL_SYSPLLCFG0_ENABLECLK1_MASK |
            SYSCTL_SYSPLLCFG0_ENABLECLK0_MASK |
            SYSCTL_SYSPLLCFG0_MCLK2XVCO_MASK));

    DL_SYSCTL_enableSYSPLL();
    return SYSCFG_DL_SYSCTL_waitClockStatus(SYSCTL_CLKSTATUS_SYSPLLGOOD_MASK,
        DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD,
        SYSCFG_DL_SYSCTL_PLL_GOOD_TIMEOUT);
}

/*
 * 作用：把 MCLK 切到 SYSPLL 所在的 HSCLK。
 * 使用场景：PLL 已经锁定后切换正式系统时钟。
 */
static bool SYSCFG_DL_SYSCTL_switchMCLKToPLLTimeout(void)
{
    DL_SYSCTL_setHSCLKSource(DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
    if (!SYSCFG_DL_SYSCTL_waitClockStatus(SYSCTL_CLKSTATUS_HSCLKGOOD_MASK,
        DL_SYSCTL_CLK_STATUS_HSCLK_GOOD,
        SYSCFG_DL_SYSCTL_HSCLK_TIMEOUT)) {
        return false;
    }

    SYSCTL->SOCLOCK.MCLKCFG |= SYSCTL_MCLKCFG_USEHSCLK_ENABLE;
    return SYSCFG_DL_SYSCTL_waitClockStatus(SYSCTL_CLKSTATUS_HSCLKMUX_MASK,
        DL_SYSCTL_CLK_STATUS_MCLK_SOURCE_HSCLK,
        SYSCFG_DL_SYSCTL_HSCLK_TIMEOUT);
}

SYSCONFIG_WEAK bool SYSCFG_DL_SYSCTL_SYSPLL_init(void)
{
    bool ratioOk = false;
    uint32_t pllCount;
    uint32_t sysoscCount;
    uint32_t ratio;
    uint32_t timeout;

    DL_SYSCTL_setFCCPeriods(DL_SYSCTL_FCC_TRIG_CNT_01);
    DL_SYSCTL_configFCC(DL_SYSCTL_FCC_TRIG_TYPE_RISE_RISE,
        DL_SYSCTL_FCC_TRIG_SOURCE_LFCLK, DL_SYSCTL_FCC_CLOCK_SOURCE_SYSPLLCLK0);
    timeout = 0;
    DL_SYSCTL_startFCC();
    while (DL_SYSCTL_isFCCDone() == 0U) {
        delay_cycles(977);
        if (++timeout > 65U) {
            break;
        }
    }
    pllCount = DL_SYSCTL_readFCC();

    DL_SYSCTL_configFCC(DL_SYSCTL_FCC_TRIG_TYPE_RISE_RISE,
        DL_SYSCTL_FCC_TRIG_SOURCE_LFCLK, DL_SYSCTL_FCC_CLOCK_SOURCE_HFCLK);
    timeout = 0;
    DL_SYSCTL_startFCC();
    while (DL_SYSCTL_isFCCDone() == 0U) {
        delay_cycles(977);
        if (++timeout > 65U) {
            break;
        }
    }
    sysoscCount = DL_SYSCTL_readFCC();

    if ((pllCount == 0U) || (sysoscCount == 0U)) {
        return false;
    }
    ratio = (pllCount * 1000U) / sysoscCount;
    if ((1994U < ratio) && (ratio < 2006U)) {
        ratioOk = true;
    }
    return ratioOk;
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
#if SYSCFG_DL_ENABLE_HFXT_PLL
    uint32_t retry;
#endif

    g_sysctlClockOk = false;
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

#if SYSCFG_DL_ENABLE_HFXT_PLL
    DL_SYSCTL_setHFCLKSourceHFXTParams(DL_SYSCTL_HFXT_RANGE_32_48_MHZ, 0, false);
    for (retry = 0U; retry < SYSCFG_DL_SYSCTL_PLL_RETRY_MAX; ++retry) {
        if (SYSCFG_DL_SYSCTL_configSYSPLLTimeout(&gSYSPLLConfig) &&
            SYSCFG_DL_SYSCTL_SYSPLL_init()) {
            DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
            if (SYSCFG_DL_SYSCTL_switchMCLKToPLLTimeout()) {
                g_sysctlClockOk = true;
                return;
            }
        }
        DL_SYSCTL_disableSYSPLL();
    }

    DL_SYSCTL_disableSYSPLL();
#else
    g_sysctlClockOk = true;
#endif
}

/*
 * 1.1ccs 起电机改为闭环步进 DIR/STEP。
 * PWM/TB6612 初始化保留空函数，避免旧代码链接名失效。
 */
SYSCONFIG_WEAK void SYSCFG_DL_PWM_init(void)
{
}

static const DL_I2C_ClockConfig gOLEDClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_OLED_init(void)
{
    DL_I2C_setClockConfig(OLED_INST, (DL_I2C_ClockConfig *) &gOLEDClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(
        OLED_INST, DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(OLED_INST);
    DL_I2C_setDigitalGlitchFilterPulseWidth(
        OLED_INST, DL_I2C_DIGITAL_GLITCH_FILTER_WIDTH_CLOCKS_1);
    DL_I2C_resetControllerTransfer(OLED_INST);
    DL_I2C_setTimerPeriod(OLED_INST, 39);
    DL_I2C_setControllerTXFIFOThreshold(
        OLED_INST, DL_I2C_TX_FIFO_LEVEL_BYTES_7);
    DL_I2C_setControllerRXFIFOThreshold(
        OLED_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_8);
    DL_I2C_enableControllerClockStretching(OLED_INST);
    DL_I2C_enableController(OLED_INST);
}

static const DL_UART_Main_ClockConfig gUART32MClockConfig = {
    .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUARTConfig = {
    .mode = DL_UART_MAIN_MODE_NORMAL,
    .direction = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity = DL_UART_MAIN_PARITY_NONE,
    .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_LogUart_init(void)
{
    DL_UART_Main_setClockConfig(
        LogUart_INST, (DL_UART_Main_ClockConfig *) &gUART32MClockConfig);
    DL_UART_Main_init(LogUart_INST, (DL_UART_Main_Config *) &gUARTConfig);
    DL_UART_Main_setOversampling(LogUart_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(
        LogUart_INST, LogUart_IBRD_32_MHZ_115200_BAUD,
        LogUart_FBRD_32_MHZ_115200_BAUD);
    DL_UART_Main_enable(LogUart_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_JY61P_init(void)
{
    DL_UART_Main_setClockConfig(
        JY61P_INST, (DL_UART_Main_ClockConfig *) &gUART32MClockConfig);
    DL_UART_Main_init(JY61P_INST, (DL_UART_Main_Config *) &gUARTConfig);
    DL_UART_Main_setOversampling(JY61P_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(
        JY61P_INST, JY61P_IBRD_32_MHZ_115200_BAUD,
        JY61P_FBRD_32_MHZ_115200_BAUD);
    DL_UART_Main_enable(JY61P_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_Exchange_init(void)
{
    DL_UART_Main_setClockConfig(
        Exchange_INST, (DL_UART_Main_ClockConfig *) &gUART32MClockConfig);
    DL_UART_Main_init(Exchange_INST, (DL_UART_Main_Config *) &gUARTConfig);
    DL_UART_Main_setOversampling(Exchange_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(
        Exchange_INST, Exchange_IBRD_32_MHZ_115200_BAUD,
        Exchange_FBRD_32_MHZ_115200_BAUD);
    DL_UART_Main_enable(Exchange_INST);
}

static const DL_ADC12_ClockConfig gADCClockConfig = {
    .clockSel = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32
};

SYSCONFIG_WEAK void SYSCFG_DL_GRAY_ADC0_init(void)
{
    DL_ADC12_setClockConfig(GRAY_ADC0_INST,
        (DL_ADC12_ClockConfig *) &gADCClockConfig);
    DL_ADC12_initSeqSample(GRAY_ADC0_INST, DL_ADC12_REPEAT_MODE_DISABLED,
        DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        ADC12_CTL2_STARTADD_ADDR_00, ADC12_CTL2_ENDADD_ADDR_03,
        DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_setSampleTime0(GRAY_ADC0_INST, 39);
    DL_ADC12_configConversionMem(GRAY_ADC0_INST, GRAY_ADC0_MEM_GRAY4,
        DL_ADC12_INPUT_CHAN_3, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(GRAY_ADC0_INST, GRAY_ADC0_MEM_GRAY5,
        DL_ADC12_INPUT_CHAN_2, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(GRAY_ADC0_INST, GRAY_ADC0_MEM_GRAY6,
        DL_ADC12_INPUT_CHAN_1, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(GRAY_ADC0_INST, GRAY_ADC0_MEM_GRAY7,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_TRIGGER_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
}

SYSCONFIG_WEAK void SYSCFG_DL_GRAY_ADC1_init(void)
{
    DL_ADC12_setClockConfig(GRAY_ADC1_INST,
        (DL_ADC12_ClockConfig *) &gADCClockConfig);
    DL_ADC12_initSeqSample(GRAY_ADC1_INST, DL_ADC12_REPEAT_MODE_DISABLED,
        DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        ADC12_CTL2_STARTADD_ADDR_00, ADC12_CTL2_ENDADD_ADDR_02,
        DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_setSampleTime0(GRAY_ADC1_INST, 39);
    DL_ADC12_configConversionMem(GRAY_ADC1_INST, GRAY_ADC1_MEM_GRAY1,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(GRAY_ADC1_INST, GRAY_ADC1_MEM_GRAY2,
        DL_ADC12_INPUT_CHAN_1, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(GRAY_ADC1_INST, GRAY_ADC1_MEM_GRAY3,
        DL_ADC12_INPUT_CHAN_2, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_TRIGGER_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
}
