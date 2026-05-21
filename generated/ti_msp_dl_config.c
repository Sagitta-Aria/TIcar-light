#include "ti_msp_dl_config.h"

SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_init();
    SYSCFG_DL_OLED_init();
    SYSCFG_DL_JY61P_init();
    SYSCFG_DL_JQ8400_init();
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
    DL_TimerA_reset(PWM_INST);
    DL_I2C_reset(OLED_INST);
    DL_UART_Main_reset(JY61P_INST);
    DL_UART_Main_reset(JQ8400_INST);
    DL_UART_Main_reset(Exchange_INST);
    DL_ADC12_reset(GRAY_ADC0_INST);
    DL_ADC12_reset(GRAY_ADC1_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_INST);
    DL_I2C_enablePower(OLED_INST);
    DL_UART_Main_enablePower(JY61P_INST);
    DL_UART_Main_enablePower(JQ8400_INST);
    DL_UART_Main_enablePower(Exchange_INST);
    DL_ADC12_enablePower(GRAY_ADC0_INST);
    DL_ADC12_enablePower(GRAY_ADC1_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_C0_IOMUX, GPIO_PWM_C0_IOMUX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_C1_IOMUX, GPIO_PWM_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIOA, GPIO_PWM_C0_PIN | GPIO_PWM_C1_PIN);

    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_OLED_IOMUX_SDA, GPIO_OLED_IOMUX_SDA_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_OLED_IOMUX_SCL, GPIO_OLED_IOMUX_SCL_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_JY61P_IOMUX_TX, GPIO_JY61P_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_JY61P_IOMUX_RX, GPIO_JY61P_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_JQ8400_IOMUX_TX, GPIO_JQ8400_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_JQ8400_IOMUX_RX, GPIO_JQ8400_IOMUX_RX_FUNC);
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

    DL_GPIO_initDigitalOutput(MOTOR_A_IN1_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_A_IN2_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_B_IN1_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_B_IN2_IOMUX);

    DL_GPIO_initDigitalInputFeatures(KEY_1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY_2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_LEFT_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENCODER_LEFT_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENCODER_RIGHT_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENCODER_RIGHT_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(MOTOR_DIR_PORT,
        MOTOR_A_IN1_PIN | MOTOR_A_IN2_PIN | MOTOR_B_IN1_PIN | MOTOR_B_IN2_PIN);
    DL_GPIO_enableOutput(MOTOR_DIR_PORT,
        MOTOR_A_IN1_PIN | MOTOR_A_IN2_PIN | MOTOR_B_IN1_PIN | MOTOR_B_IN2_PIN);

    DL_GPIO_setLowerPinsPolarity(KEY_PORT,
        DL_GPIO_PIN_9_EDGE_RISE | DL_GPIO_PIN_8_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(KEY_PORT, KEY_1_PIN | KEY_2_PIN);
    DL_GPIO_enableInterrupt(KEY_PORT, KEY_1_PIN | KEY_2_PIN);

    DL_GPIO_setLowerPinsPolarity(ENCODER_PORT,
        DL_GPIO_PIN_12_EDGE_RISE | DL_GPIO_PIN_13_EDGE_RISE);
    DL_GPIO_setUpperPinsPolarity(ENCODER_PORT,
        DL_GPIO_PIN_22_EDGE_RISE | DL_GPIO_PIN_23_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(ENCODER_PORT,
        ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN |
        ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);
    DL_GPIO_enableInterrupt(ENCODER_PORT,
        ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN |
        ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);
}

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

    ratio = (pllCount * 1000U) / sysoscCount;
    if ((1994U < ratio) && (ratio < 2006U)) {
        ratioOk = true;
    }
    return ratioOk;
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setHFCLKSourceHFXTParams(DL_SYSCTL_HFXT_RANGE_32_48_MHZ, 0, false);
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);

    while (SYSCFG_DL_SYSCTL_SYSPLL_init() == false) {
        DL_SYSCTL_disableSYSPLL();
        DL_SYSCTL_enableSYSPLL();
        while ((DL_SYSCTL_getClockStatus() & SYSCTL_CLKSTATUS_SYSPLLGOOD_MASK) !=
            DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD) {
        }
    }
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
}

static const DL_TimerA_ClockConfig gPWMClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gPWMConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = PWM_PERIOD_COUNTS,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_init(void)
{
    DL_TimerA_setClockConfig(PWM_INST, (DL_TimerA_ClockConfig *) &gPWMClockConfig);
    DL_TimerA_initPWMMode(PWM_INST, (DL_TimerA_PWMConfig *) &gPWMConfig);
    DL_TimerA_setCounterControl(PWM_INST, DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND, DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_INST, 0U, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_INST, 0U, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(PWM_INST);
    DL_TimerA_setCCPDirection(PWM_INST,
        DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
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

static const DL_UART_Main_ClockConfig gUART40MClockConfig = {
    .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_ClockConfig gUART80MClockConfig = {
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

SYSCONFIG_WEAK void SYSCFG_DL_JY61P_init(void)
{
    DL_UART_Main_setClockConfig(
        JY61P_INST, (DL_UART_Main_ClockConfig *) &gUART40MClockConfig);
    DL_UART_Main_init(JY61P_INST, (DL_UART_Main_Config *) &gUARTConfig);
    DL_UART_Main_setOversampling(JY61P_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(
        JY61P_INST, JY61P_IBRD_40_MHZ_115200_BAUD,
        JY61P_FBRD_40_MHZ_115200_BAUD);
    DL_UART_Main_enable(JY61P_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_JQ8400_init(void)
{
    DL_UART_Main_setClockConfig(
        JQ8400_INST, (DL_UART_Main_ClockConfig *) &gUART40MClockConfig);
    DL_UART_Main_init(JQ8400_INST, (DL_UART_Main_Config *) &gUARTConfig);
    DL_UART_Main_setOversampling(JQ8400_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(
        JQ8400_INST, JQ8400_IBRD_40_MHZ_115200_BAUD,
        JQ8400_FBRD_40_MHZ_115200_BAUD);
    DL_UART_Main_enable(JQ8400_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_Exchange_init(void)
{
    DL_UART_Main_setClockConfig(
        Exchange_INST, (DL_UART_Main_ClockConfig *) &gUART80MClockConfig);
    DL_UART_Main_init(Exchange_INST, (DL_UART_Main_Config *) &gUARTConfig);
    DL_UART_Main_setOversampling(Exchange_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(
        Exchange_INST, Exchange_IBRD_80_MHZ_115200_BAUD,
        Exchange_FBRD_80_MHZ_115200_BAUD);
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
    DL_ADC12_configConversionMem(GRAY_ADC0_INST, GRAY_ADC0_MEM_GRAY1,
        DL_ADC12_INPUT_CHAN_12, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(GRAY_ADC0_INST, GRAY_ADC0_MEM_GRAY6,
        DL_ADC12_INPUT_CHAN_3, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(GRAY_ADC0_INST, GRAY_ADC0_MEM_GRAY7,
        DL_ADC12_INPUT_CHAN_2, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(GRAY_ADC0_INST, GRAY_ADC0_MEM_GRAY5,
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
    DL_ADC12_configConversionMem(GRAY_ADC1_INST, GRAY_ADC1_MEM_GRAY2,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(GRAY_ADC1_INST, GRAY_ADC1_MEM_GRAY3,
        DL_ADC12_INPUT_CHAN_1, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(GRAY_ADC1_INST, GRAY_ADC1_MEM_GRAY4,
        DL_ADC12_INPUT_CHAN_2, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_TRIGGER_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
}
