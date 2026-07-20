#include "encoder_motor.h"

#include "board_config.h"
#include "control_config.h"
#include "pin_map.h"

typedef struct {
    int16_t targetCps;
    int16_t commandCounts;
    int16_t openLoopPwm;
    int16_t pwm;
    int32_t targetCounts;
    int32_t feedbackCounts;
    int64_t integralScaled;
    uint8_t startupActive;
    uint8_t zeroTargetBrakeEnabled;
} EncoderMotorController;

static EncoderMotorController g_controller[ENCODER_MOTOR_COUNT];
static EncoderMotorTuning g_tuning[ENCODER_MOTOR_COUNT];
static volatile int32_t g_totalCount[ENCODER_MOTOR_COUNT];
static volatile int32_t g_intervalCount[ENCODER_MOTOR_COUNT];
static volatile uint8_t g_encoderState[ENCODER_MOTOR_COUNT];
static volatile uint32_t g_rightEncoderAInterruptCount;
static volatile uint32_t g_rightEncoderBInterruptCount;
static EncoderMotorMode g_mode;
static uint32_t g_sampleSequence;
static int32_t g_crossPwmSyncGainQ1024;
static uint16_t g_crossPwmSyncLimit;

static void EncoderMotor_WritePwm(uint8_t motorIndex, int16_t pwm);

static const int8_t g_quadratureDelta[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint32_t EncoderMotor_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void EncoderMotor_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static int16_t EncoderMotor_ClampTarget(int32_t value)
{
    if (value > (int32_t)CHASSIS_TARGET_LIMIT_CPS) {
        value = (int32_t)CHASSIS_TARGET_LIMIT_CPS;
    } else if (value < -(int32_t)CHASSIS_TARGET_LIMIT_CPS) {
        value = -(int32_t)CHASSIS_TARGET_LIMIT_CPS;
    }
    return (int16_t)value;
}

static int16_t EncoderMotor_ClampPeriodTarget(int32_t counts)
{
    if (counts > (int32_t)CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) {
        counts = (int32_t)CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD;
    } else if (counts <
        -(int32_t)CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) {
        counts = -(int32_t)CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD;
    }
    return (int16_t)counts;
}

/*
 * 左右反馈围绕平均速度生成等量反向修正，保持原始平均目标不变。
 * 只由正常闭环的同速命令调用；修正不会让任一目标越过零点。
 */
static int32_t EncoderMotor_CalculateStraightSyncCorrection(
    int32_t leftFeedback, int32_t rightFeedback, int32_t baseTarget)
{
    int64_t scaledDifference;
    int64_t divisor = 2LL * CHASSIS_Q1024_SCALE;
    int32_t correction;
    int32_t directionLimit;
    int32_t targetHeadroom;

    if ((baseTarget == 0) ||
        (CHASSIS_STRAIGHT_SYNC_GAIN_Q1024 == 0L)) {
        return 0;
    }

    scaledDifference = ((int64_t)leftFeedback -
        (int64_t)rightFeedback) * CHASSIS_STRAIGHT_SYNC_GAIN_Q1024;
    correction = (int32_t)(scaledDifference / divisor);

    if (correction >
        (int32_t)CHASSIS_STRAIGHT_SYNC_LIMIT_COUNTS_PER_PERIOD) {
        correction =
            (int32_t)CHASSIS_STRAIGHT_SYNC_LIMIT_COUNTS_PER_PERIOD;
    } else if (correction <
        -(int32_t)CHASSIS_STRAIGHT_SYNC_LIMIT_COUNTS_PER_PERIOD) {
        correction =
            -(int32_t)CHASSIS_STRAIGHT_SYNC_LIMIT_COUNTS_PER_PERIOD;
    }

    directionLimit = (baseTarget > 0) ? baseTarget - 1 :
        -baseTarget - 1;
    targetHeadroom = (int32_t)CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD -
        ((baseTarget > 0) ? baseTarget : -baseTarget);
    if (directionLimit > targetHeadroom) {
        directionLimit = targetHeadroom;
    }
    if (correction > directionLimit) {
        correction = directionLimit;
    } else if (correction < -directionLimit) {
        correction = -directionLimit;
    }
    return correction;
}

/* 同向直接PWM时按左右速度差交叉分配补偿；异向或单轮命令不做同步。 */
static int32_t EncoderMotor_CalculateCrossPwmSyncCorrection(
    int32_t leftFeedback, int32_t rightFeedback, int16_t leftPwm,
    int16_t rightPwm, int32_t syncGainQ1024, uint16_t syncLimitPwm)
{
    int64_t correction;

    if (((int32_t)leftPwm * (int32_t)rightPwm <= 0) ||
        (syncGainQ1024 == 0L)) {
        return 0;
    }

    correction = ((int64_t)leftFeedback - (int64_t)rightFeedback) *
        syncGainQ1024 / CHASSIS_Q1024_SCALE;
    if (correction > (int64_t)syncLimitPwm) {
        correction = (int64_t)syncLimitPwm;
    } else if (correction < -(int64_t)syncLimitPwm) {
        correction = -(int64_t)syncLimitPwm;
    }
    return (int32_t)correction;
}

static void EncoderMotor_ApplyTargetLocked(uint8_t motorIndex,
    int16_t targetCps, int16_t commandCounts)
{
    EncoderMotorController *controller = &g_controller[motorIndex];

    if (((int32_t)controller->commandCounts * commandCounts) < 0) {
        controller->integralScaled = 0;
    }
    if (commandCounts == 0) {
        controller->startupActive = 0U;
    } else if ((controller->commandCounts == 0) ||
        (((int32_t)controller->commandCounts * commandCounts) < 0)) {
        controller->startupActive = 1U;
    }
    controller->targetCps = targetCps;
    controller->commandCounts = commandCounts;
    controller->targetCounts = commandCounts;
    if (commandCounts == 0) {
        controller->integralScaled = 0;
        controller->pwm = 0;
        EncoderMotor_WritePwm(motorIndex, 0);
    }
}

static int64_t EncoderMotor_ClampIntegralScaled(uint8_t motorIndex,
    int64_t integralScaled)
{
    int64_t limitScaled =
        (int64_t)g_tuning[motorIndex].integralLimitPwm *
        CHASSIS_Q1024_SCALE;

    if (integralScaled > limitScaled) {
        return limitScaled;
    }
    if (integralScaled < -limitScaled) {
        return -limitScaled;
    }
    return integralScaled;
}

/* 只在整车由停止进入同向行驶时预装一次左右反相积分补偿。 */
static void EncoderMotor_ArmStartupCompensationLocked(int16_t leftCounts,
    int16_t rightCounts)
{
    uint8_t chassisStopped =
        ((g_controller[ENCODER_MOTOR_LEFT].commandCounts == 0) &&
        (g_controller[ENCODER_MOTOR_RIGHT].commandCounts == 0)) ? 1U : 0U;
    int64_t leftCompensation;
    int64_t rightCompensation;

    if ((chassisStopped == 0U) ||
        (((int32_t)leftCounts * (int32_t)rightCounts) <= 0)) {
        return;
    }

    leftCompensation =
        (int64_t)CHASSIS_LEFT_STARTUP_COMPENSATION_PWM_COUNTS *
        CHASSIS_Q1024_SCALE;
    rightCompensation = -leftCompensation;
    if (leftCounts < 0) {
        leftCompensation = -leftCompensation;
    }
    if (rightCounts < 0) {
        rightCompensation = -rightCompensation;
    }

    g_controller[ENCODER_MOTOR_LEFT].integralScaled =
        EncoderMotor_ClampIntegralScaled(
            ENCODER_MOTOR_LEFT, leftCompensation);
    g_controller[ENCODER_MOTOR_RIGHT].integralScaled =
        EncoderMotor_ClampIntegralScaled(
            ENCODER_MOTOR_RIGHT, rightCompensation);
}

static int16_t EncoderMotor_ClampPwm(int64_t value)
{
    if (value > (int64_t)CHASSIS_PWM_LIMIT_COUNTS) {
        value = CHASSIS_PWM_LIMIT_COUNTS;
    } else if (value < -(int64_t)CHASSIS_PWM_LIMIT_COUNTS) {
        value = -(int64_t)CHASSIS_PWM_LIMIT_COUNTS;
    }
    return (int16_t)value;
}

/* 目标为0时只按当前编码速度生成反向阻尼，不使用起步、FF或积分项。 */
static int16_t EncoderMotor_CalculateZeroTargetBrakePwm(int32_t feedback)
{
    int64_t feedbackMagnitude = (feedback < 0) ? -(int64_t)feedback :
        (int64_t)feedback;
    int64_t pwm;

    if ((CHASSIS_ZERO_TARGET_BRAKE_ENABLE == 0U) ||
        (feedbackMagnitude <= (int64_t)
            CHASSIS_ZERO_TARGET_BRAKE_DEADBAND_COUNTS_PER_PERIOD)) {
        return 0;
    }

    pwm = -((int64_t)feedback * CHASSIS_ZERO_TARGET_BRAKE_KP_Q1024) /
        CHASSIS_Q1024_SCALE;
    if (pwm > (int64_t)CHASSIS_ZERO_TARGET_BRAKE_PWM_LIMIT_COUNTS) {
        pwm = (int64_t)CHASSIS_ZERO_TARGET_BRAKE_PWM_LIMIT_COUNTS;
    } else if (pwm <
        -(int64_t)CHASSIS_ZERO_TARGET_BRAKE_PWM_LIMIT_COUNTS) {
        pwm = -(int64_t)CHASSIS_ZERO_TARGET_BRAKE_PWM_LIMIT_COUNTS;
    }
    return (int16_t)pwm;
}

static uint8_t EncoderMotor_ReadState(uint8_t motorIndex)
{
    uint8_t state = 0U;

    if (motorIndex == ENCODER_MOTOR_LEFT) {
        if ((DL_GPIO_readPins(PIN_CHASSIS_LEFT_ENCODER_A_PORT,
            PIN_CHASSIS_LEFT_ENCODER_A) &
            PIN_CHASSIS_LEFT_ENCODER_A) != 0U) {
            state |= 2U;
        }
        if ((DL_GPIO_readPins(PIN_CHASSIS_LEFT_ENCODER_B_PORT,
            PIN_CHASSIS_LEFT_ENCODER_B) &
            PIN_CHASSIS_LEFT_ENCODER_B) != 0U) {
            state |= 1U;
        }
    } else {
        if ((DL_GPIO_readPins(PIN_CHASSIS_RIGHT_ENCODER_A_PORT,
            PIN_CHASSIS_RIGHT_ENCODER_A) &
            PIN_CHASSIS_RIGHT_ENCODER_A) != 0U) {
            state |= 2U;
        }
        if ((DL_GPIO_readPins(PIN_CHASSIS_RIGHT_ENCODER_B_PORT,
            PIN_CHASSIS_RIGHT_ENCODER_B) &
            PIN_CHASSIS_RIGHT_ENCODER_B) != 0U) {
            state |= 1U;
        }
    }
    return state;
}

static void EncoderMotor_UpdateEncoder(uint8_t motorIndex)
{
    uint8_t current = EncoderMotor_ReadState(motorIndex);
    uint8_t transition = (uint8_t)((g_encoderState[motorIndex] << 2U) |
        current);
    int32_t delta = g_quadratureDelta[transition & 0x0FU];

    if (motorIndex == ENCODER_MOTOR_LEFT) {
        delta *= CHASSIS_LEFT_ENCODER_SIGN;
    } else {
        delta *= CHASSIS_RIGHT_ENCODER_SIGN;
    }
    g_encoderState[motorIndex] = current;
    g_totalCount[motorIndex] += delta;
    g_intervalCount[motorIndex] += delta;
}

static void EncoderMotor_SetDirection(uint8_t motorIndex, int16_t pwm)
{
    uint8_t forward = (pwm > 0) ? 1U : 0U;

    if (motorIndex == ENCODER_MOTOR_LEFT) {
        if (CAR_CHASSIS_LEFT_REVERSE != 0U) {
            forward = (forward == 0U) ? 1U : 0U;
        }
        if (pwm == 0) {
            DL_GPIO_clearPins(GPIOA,
                PIN_CHASSIS_LEFT_IN1 | PIN_CHASSIS_LEFT_IN2);
        } else if (forward != 0U) {
            DL_GPIO_setPins(PIN_CHASSIS_LEFT_IN1_PORT,
                PIN_CHASSIS_LEFT_IN1);
            DL_GPIO_clearPins(PIN_CHASSIS_LEFT_IN2_PORT,
                PIN_CHASSIS_LEFT_IN2);
        } else {
            DL_GPIO_clearPins(PIN_CHASSIS_LEFT_IN1_PORT,
                PIN_CHASSIS_LEFT_IN1);
            DL_GPIO_setPins(PIN_CHASSIS_LEFT_IN2_PORT,
                PIN_CHASSIS_LEFT_IN2);
        }
    } else {
        if (CAR_CHASSIS_RIGHT_REVERSE != 0U) {
            forward = (forward == 0U) ? 1U : 0U;
        }
        if (pwm == 0) {
            DL_GPIO_clearPins(GPIOA,
                PIN_CHASSIS_RIGHT_IN1 | PIN_CHASSIS_RIGHT_IN2);
        } else if (forward != 0U) {
            DL_GPIO_setPins(PIN_CHASSIS_RIGHT_IN1_PORT,
                PIN_CHASSIS_RIGHT_IN1);
            DL_GPIO_clearPins(PIN_CHASSIS_RIGHT_IN2_PORT,
                PIN_CHASSIS_RIGHT_IN2);
        } else {
            DL_GPIO_clearPins(PIN_CHASSIS_RIGHT_IN1_PORT,
                PIN_CHASSIS_RIGHT_IN1);
            DL_GPIO_setPins(PIN_CHASSIS_RIGHT_IN2_PORT,
                PIN_CHASSIS_RIGHT_IN2);
        }
    }
}

static void EncoderMotor_WritePwm(uint8_t motorIndex, int16_t pwm)
{
    uint32_t duty = (pwm < 0) ?
        (uint32_t)(-(int32_t)pwm) : (uint32_t)pwm;
    uint32_t compare;

    EncoderMotor_SetDirection(motorIndex, pwm);
    if (duty > CHASSIS_PWM_HARDWARE_MAX_COUNTS) {
        duty = CHASSIS_PWM_HARDWARE_MAX_COUNTS;
    }
    compare = CHASSIS_PWM_PERIOD_COUNTS - duty;
    DL_TimerA_setCaptureCompareValue(TIMA0, compare,
        (motorIndex == ENCODER_MOTOR_LEFT) ?
            PIN_CHASSIS_LEFT_PWM_CC_INDEX : PIN_CHASSIS_RIGHT_PWM_CC_INDEX);
}

static void EncoderMotor_InitPwm(void)
{
    DL_TimerA_ClockConfig clockConfig = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U
    };
    DL_TimerA_PWMConfig pwmConfig = {
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period = CHASSIS_PWM_PERIOD_COUNTS,
        .isTimerWithFourCC = true,
        .startTimer = DL_TIMER_STOP
    };

    DL_TimerA_reset(TIMA0);
    DL_TimerA_enablePower(TIMA0);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_GPIO_initPeripheralOutputFunction(PIN_CHASSIS_LEFT_PWM_IOMUX,
        PIN_CHASSIS_LEFT_PWM_FUNC);
    DL_GPIO_initPeripheralOutputFunction(PIN_CHASSIS_RIGHT_PWM_IOMUX,
        PIN_CHASSIS_RIGHT_PWM_FUNC);
    DL_GPIO_enableOutput(PIN_CHASSIS_LEFT_PWM_PORT,
        PIN_CHASSIS_LEFT_PWM);
    DL_GPIO_enableOutput(PIN_CHASSIS_RIGHT_PWM_PORT,
        PIN_CHASSIS_RIGHT_PWM);

    DL_TimerA_setClockConfig(TIMA0, &clockConfig);
    DL_TimerA_initPWMMode(TIMA0, &pwmConfig);
    DL_TimerA_setCounterControl(TIMA0, DL_TIMER_CZC_CCCTL1_ZCOND,
        DL_TIMER_CAC_CCCTL1_ACOND, DL_TIMER_CLC_CCCTL1_LCOND);
    DL_TimerA_setCaptureCompareOutCtl(TIMA0,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareOutCtl(TIMA0,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL, DL_TIMERA_CAPTURE_COMPARE_3_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(TIMA0,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(TIMA0,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, DL_TIMERA_CAPTURE_COMPARE_3_INDEX);
    DL_TimerA_setCaptureCompareValue(TIMA0, CHASSIS_PWM_PERIOD_COUNTS,
        PIN_CHASSIS_LEFT_PWM_CC_INDEX);
    DL_TimerA_setCaptureCompareValue(TIMA0, CHASSIS_PWM_PERIOD_COUNTS,
        PIN_CHASSIS_RIGHT_PWM_CC_INDEX);
    DL_TimerA_enableClock(TIMA0);
    DL_TimerA_setCCPDirection(TIMA0,
        DL_TIMER_CC1_OUTPUT | DL_TIMER_CC3_OUTPUT);
    DL_TimerA_startCounter(TIMA0);
}

static void EncoderMotor_InitDirectionPins(void)
{
    DL_GPIO_initDigitalOutput(PIN_CHASSIS_LEFT_IN1_IOMUX);
    DL_GPIO_initDigitalOutput(PIN_CHASSIS_LEFT_IN2_IOMUX);
    DL_GPIO_initDigitalOutput(PIN_CHASSIS_RIGHT_IN1_IOMUX);
    DL_GPIO_initDigitalOutput(PIN_CHASSIS_RIGHT_IN2_IOMUX);
    DL_GPIO_clearPins(GPIOA,
        PIN_CHASSIS_LEFT_IN1 | PIN_CHASSIS_LEFT_IN2 |
        PIN_CHASSIS_RIGHT_IN1 | PIN_CHASSIS_RIGHT_IN2);
    DL_GPIO_enableOutput(GPIOA,
        PIN_CHASSIS_LEFT_IN1 | PIN_CHASSIS_LEFT_IN2 |
        PIN_CHASSIS_RIGHT_IN1 | PIN_CHASSIS_RIGHT_IN2);
}

static void EncoderMotor_InitEncoderPins(void)
{
    DL_GPIO_initDigitalInputFeatures(PIN_CHASSIS_LEFT_ENCODER_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(PIN_CHASSIS_LEFT_ENCODER_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(PIN_CHASSIS_RIGHT_ENCODER_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(PIN_CHASSIS_RIGHT_ENCODER_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setLowerPinsPolarity(GPIOA, DL_GPIO_PIN_13_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOB,
        DL_GPIO_PIN_19_EDGE_RISE_FALL | DL_GPIO_PIN_20_EDGE_RISE_FALL |
        DL_GPIO_PIN_24_EDGE_RISE_FALL);

    DL_GPIO_disableInterrupt(PIN_CHASSIS_LEFT_ENCODER_A_PORT,
        PIN_CHASSIS_LEFT_ENCODER_A);
    DL_GPIO_disableInterrupt(PIN_CHASSIS_LEFT_ENCODER_B_PORT,
        PIN_CHASSIS_LEFT_ENCODER_B);
    DL_GPIO_disableInterrupt(PIN_CHASSIS_RIGHT_ENCODER_A_PORT,
        PIN_CHASSIS_RIGHT_ENCODER_A);
    DL_GPIO_disableInterrupt(PIN_CHASSIS_RIGHT_ENCODER_B_PORT,
        PIN_CHASSIS_RIGHT_ENCODER_B);
    DL_GPIO_clearInterruptStatus(PIN_CHASSIS_LEFT_ENCODER_A_PORT,
        PIN_CHASSIS_LEFT_ENCODER_A);
    DL_GPIO_clearInterruptStatus(PIN_CHASSIS_LEFT_ENCODER_B_PORT,
        PIN_CHASSIS_LEFT_ENCODER_B);
    DL_GPIO_clearInterruptStatus(PIN_CHASSIS_RIGHT_ENCODER_A_PORT,
        PIN_CHASSIS_RIGHT_ENCODER_A);
    DL_GPIO_clearInterruptStatus(PIN_CHASSIS_RIGHT_ENCODER_B_PORT,
        PIN_CHASSIS_RIGHT_ENCODER_B);
}

static void EncoderMotor_EnableEncoderInterrupts(void)
{
    /* Discard edges that occurred while the initial AB state was sampled. */
    DL_GPIO_clearInterruptStatus(PIN_CHASSIS_LEFT_ENCODER_A_PORT,
        PIN_CHASSIS_LEFT_ENCODER_A);
    DL_GPIO_clearInterruptStatus(PIN_CHASSIS_LEFT_ENCODER_B_PORT,
        PIN_CHASSIS_LEFT_ENCODER_B);
    DL_GPIO_clearInterruptStatus(PIN_CHASSIS_RIGHT_ENCODER_A_PORT,
        PIN_CHASSIS_RIGHT_ENCODER_A);
    DL_GPIO_clearInterruptStatus(PIN_CHASSIS_RIGHT_ENCODER_B_PORT,
        PIN_CHASSIS_RIGHT_ENCODER_B);
    DL_GPIO_enableInterrupt(PIN_CHASSIS_LEFT_ENCODER_A_PORT,
        PIN_CHASSIS_LEFT_ENCODER_A);
    DL_GPIO_enableInterrupt(PIN_CHASSIS_LEFT_ENCODER_B_PORT,
        PIN_CHASSIS_LEFT_ENCODER_B);
    DL_GPIO_enableInterrupt(PIN_CHASSIS_RIGHT_ENCODER_A_PORT,
        PIN_CHASSIS_RIGHT_ENCODER_A);
    DL_GPIO_enableInterrupt(PIN_CHASSIS_RIGHT_ENCODER_B_PORT,
        PIN_CHASSIS_RIGHT_ENCODER_B);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void EncoderMotor_Init(void)
{
    uint8_t index;

    EncoderMotor_InitDirectionPins();
    EncoderMotor_InitPwm();
    EncoderMotor_InitEncoderPins();
    for (index = 0U; index < ENCODER_MOTOR_COUNT; ++index) {
        g_controller[index].targetCps = 0;
        g_controller[index].commandCounts = 0;
        g_controller[index].openLoopPwm = 0;
        g_controller[index].pwm = 0;
        g_controller[index].targetCounts = 0;
        g_controller[index].feedbackCounts = 0;
        g_controller[index].integralScaled = 0;
        g_controller[index].startupActive = 0U;
        g_controller[index].zeroTargetBrakeEnabled = 0U;
        g_totalCount[index] = 0;
        g_intervalCount[index] = 0;
        g_encoderState[index] = EncoderMotor_ReadState(index);
    }
    g_rightEncoderAInterruptCount = 0U;
    g_rightEncoderBInterruptCount = 0U;
    g_tuning[ENCODER_MOTOR_LEFT].startPwm =
        CHASSIS_LEFT_START_PWM_COUNTS;
    g_tuning[ENCODER_MOTOR_LEFT].runStartPwm =
        CHASSIS_LEFT_RUN_START_PWM_COUNTS;
    g_tuning[ENCODER_MOTOR_LEFT].integralLimitPwm =
        CHASSIS_LEFT_INTEGRAL_LIMIT_PWM_COUNTS;
    g_tuning[ENCODER_MOTOR_LEFT].ffQ1024 = CHASSIS_LEFT_FF_Q1024;
    g_tuning[ENCODER_MOTOR_LEFT].kpQ1024 = CHASSIS_LEFT_KP_Q1024;
    g_tuning[ENCODER_MOTOR_LEFT].kiQ1024 = CHASSIS_LEFT_KI_Q1024;
    g_tuning[ENCODER_MOTOR_RIGHT].startPwm =
        CHASSIS_RIGHT_START_PWM_COUNTS;
    g_tuning[ENCODER_MOTOR_RIGHT].runStartPwm =
        CHASSIS_RIGHT_RUN_START_PWM_COUNTS;
    g_tuning[ENCODER_MOTOR_RIGHT].integralLimitPwm =
        CHASSIS_RIGHT_INTEGRAL_LIMIT_PWM_COUNTS;
    g_tuning[ENCODER_MOTOR_RIGHT].ffQ1024 = CHASSIS_RIGHT_FF_Q1024;
    g_tuning[ENCODER_MOTOR_RIGHT].kpQ1024 = CHASSIS_RIGHT_KP_Q1024;
    g_tuning[ENCODER_MOTOR_RIGHT].kiQ1024 = CHASSIS_RIGHT_KI_Q1024;
    g_mode = ENCODER_MOTOR_MODE_NORMAL;
    g_sampleSequence = 0U;
    g_crossPwmSyncGainQ1024 = 0L;
    g_crossPwmSyncLimit = 0U;
    EncoderMotor_EnableEncoderInterrupts();
    EncoderMotor_Stop();
}

void EncoderMotor_SetTargets(int16_t leftCps, int16_t rightCps)
{
    int16_t nextLeft = EncoderMotor_ClampTarget(leftCps);
    int16_t nextRight = EncoderMotor_ClampTarget(rightCps);
    int16_t leftCounts =
        (int16_t)(nextLeft / (int16_t)CHASSIS_SPEED_UNIT_HZ);
    int16_t rightCounts =
        (int16_t)(nextRight / (int16_t)CHASSIS_SPEED_UNIT_HZ);
    uint32_t primask = EncoderMotor_EnterCritical();

    EncoderMotor_ArmStartupCompensationLocked(leftCounts, rightCounts);
    EncoderMotor_ApplyTargetLocked(ENCODER_MOTOR_LEFT, nextLeft, leftCounts);
    EncoderMotor_ApplyTargetLocked(ENCODER_MOTOR_RIGHT, nextRight,
        rightCounts);
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_SetPeriodTargets(int16_t leftCounts, int16_t rightCounts)
{
    int16_t nextLeft = EncoderMotor_ClampPeriodTarget(leftCounts);
    int16_t nextRight = EncoderMotor_ClampPeriodTarget(rightCounts);
    uint32_t primask = EncoderMotor_EnterCritical();

    g_mode = ENCODER_MOTOR_MODE_NORMAL;
    EncoderMotor_ArmStartupCompensationLocked(nextLeft, nextRight);
    EncoderMotor_ApplyTargetLocked(ENCODER_MOTOR_LEFT,
        (int16_t)(nextLeft * (int16_t)CHASSIS_SPEED_UNIT_HZ), nextLeft);
    EncoderMotor_ApplyTargetLocked(ENCODER_MOTOR_RIGHT,
        (int16_t)(nextRight * (int16_t)CHASSIS_SPEED_UNIT_HZ), nextRight);
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_SetCrossCoupledPwm(int16_t leftPwm, int16_t rightPwm,
    int32_t syncGainQ1024, uint16_t syncLimitPwm)
{
    uint32_t primask = EncoderMotor_EnterCritical();

    g_mode = ENCODER_MOTOR_MODE_CROSS_COUPLED_PWM;
    g_crossPwmSyncGainQ1024 = (syncGainQ1024 < 0L) ? 0L : syncGainQ1024;
    g_crossPwmSyncLimit = (syncLimitPwm > CHASSIS_PWM_LIMIT_COUNTS) ?
        (uint16_t)CHASSIS_PWM_LIMIT_COUNTS : syncLimitPwm;
    g_controller[ENCODER_MOTOR_LEFT].openLoopPwm =
        EncoderMotor_ClampPwm(leftPwm);
    g_controller[ENCODER_MOTOR_RIGHT].openLoopPwm =
        EncoderMotor_ClampPwm(rightPwm);
    g_controller[ENCODER_MOTOR_LEFT].targetCps = 0;
    g_controller[ENCODER_MOTOR_RIGHT].targetCps = 0;
    g_controller[ENCODER_MOTOR_LEFT].commandCounts = 0;
    g_controller[ENCODER_MOTOR_RIGHT].commandCounts = 0;
    g_controller[ENCODER_MOTOR_LEFT].targetCounts = 0;
    g_controller[ENCODER_MOTOR_RIGHT].targetCounts = 0;
    g_controller[ENCODER_MOTOR_LEFT].integralScaled = 0;
    g_controller[ENCODER_MOTOR_RIGHT].integralScaled = 0;
    g_controller[ENCODER_MOTOR_LEFT].startupActive = 0U;
    g_controller[ENCODER_MOTOR_RIGHT].startupActive = 0U;
    g_controller[ENCODER_MOTOR_LEFT].zeroTargetBrakeEnabled = 0U;
    g_controller[ENCODER_MOTOR_RIGHT].zeroTargetBrakeEnabled = 0U;
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_SetZeroTargetBrake(uint8_t motorIndex, uint8_t enabled)
{
    EncoderMotorController *controller;
    uint32_t primask;

    if (motorIndex >= ENCODER_MOTOR_COUNT) {
        return;
    }

    primask = EncoderMotor_EnterCritical();
    controller = &g_controller[motorIndex];
    controller->zeroTargetBrakeEnabled = (enabled != 0U) ? 1U : 0U;
    if ((controller->zeroTargetBrakeEnabled == 0U) &&
        (controller->commandCounts == 0)) {
        controller->integralScaled = 0;
        controller->pwm = 0;
        EncoderMotor_WritePwm(motorIndex, 0);
    }
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_SetTarget(uint8_t motorIndex, int16_t targetCps)
{
    uint32_t primask;

    if (motorIndex >= ENCODER_MOTOR_COUNT) {
        return;
    }
    targetCps = EncoderMotor_ClampTarget(targetCps);
    primask = EncoderMotor_EnterCritical();
    EncoderMotor_ApplyTargetLocked(motorIndex, targetCps,
        (int16_t)(targetCps / (int16_t)CHASSIS_SPEED_UNIT_HZ));
    EncoderMotor_ExitCritical(primask);
}

static void EncoderMotor_ReadAndClearIntervals(int32_t *left, int32_t *right)
{
    uint32_t primask = EncoderMotor_EnterCritical();

    /* 10 ms原始窗口换算为固定count/20ms速度刻度，保留现有标定参数。 */
    *left = g_intervalCount[ENCODER_MOTOR_LEFT] *
        (int32_t)CHASSIS_FEEDBACK_COUNT_SCALE;
    *right = g_intervalCount[ENCODER_MOTOR_RIGHT] *
        (int32_t)CHASSIS_FEEDBACK_COUNT_SCALE;
    g_intervalCount[ENCODER_MOTOR_LEFT] = 0;
    g_intervalCount[ENCODER_MOTOR_RIGHT] = 0;
    EncoderMotor_ExitCritical(primask);
}

static int16_t EncoderMotor_UpdateClosedLoop(uint8_t motorIndex,
    int32_t targetCounts, int32_t feedback)
{
    EncoderMotorController *controller = &g_controller[motorIndex];
    const EncoderMotorTuning *tuning = &g_tuning[motorIndex];
    int32_t error;
    int64_t outputScaled;
    int64_t feedbackMagnitude;
    int32_t selectedStartPwm;
    int32_t startScaled;

    controller->targetCounts = targetCounts;
    controller->feedbackCounts = feedback;
    if (targetCounts == 0) {
        controller->integralScaled = 0;
        controller->startupActive = 0U;
        controller->pwm = (controller->zeroTargetBrakeEnabled != 0U) ?
            EncoderMotor_CalculateZeroTargetBrakePwm(feedback) : 0;
        return controller->pwm;
    }
    feedbackMagnitude = (feedback < 0) ? -(int64_t)feedback :
        (int64_t)feedback;
    /*
     * START只负责本次起步：达到运行速度后单向切到RUN_START。
     * 运行中即使反馈在阈值附近波动，也不能重新切回START，否则两套
     * 基础PWM的台阶会直接叠加到速度环输出，造成低速循迹左右抢速。
     * 停车或目标反向时由EncoderMotor_ApplyTargetLocked重新进入START。
     */
    if ((controller->startupActive != 0U) &&
        (feedbackMagnitude >=
            (int64_t)CHASSIS_START_PWM_SPEED_THRESHOLD_COUNTS_PER_PERIOD)) {
        controller->startupActive = 0U;
    }
    error = targetCounts - feedback;
    controller->integralScaled +=
        ((int64_t)tuning->kiQ1024 * error *
            (int64_t)CHASSIS_CONTROL_PERIOD_MS) /
        (int64_t)CHASSIS_SPEED_UNIT_PERIOD_MS;
    controller->integralScaled = EncoderMotor_ClampIntegralScaled(
        motorIndex, controller->integralScaled);
    selectedStartPwm = (controller->startupActive != 0U) ?
        tuning->startPwm : tuning->runStartPwm;
    startScaled = (targetCounts > 0) ? selectedStartPwm :
        -selectedStartPwm;
    outputScaled = (int64_t)startScaled * CHASSIS_Q1024_SCALE +
        (int64_t)tuning->ffQ1024 * targetCounts +
        (int64_t)tuning->kpQ1024 * error + controller->integralScaled;
    controller->pwm = EncoderMotor_ClampPwm(
        outputScaled / CHASSIS_Q1024_SCALE);
    return controller->pwm;
}

void EncoderMotor_RunControlPeriod(void)
{
    int32_t leftFeedback;
    int32_t rightFeedback;
    int32_t targetCounts[ENCODER_MOTOR_COUNT];
    int32_t straightSyncCorrection;
    int16_t output[ENCODER_MOTOR_COUNT];
    uint32_t primask;

    EncoderMotor_ReadAndClearIntervals(&leftFeedback, &rightFeedback);

    primask = EncoderMotor_EnterCritical();
    if (g_mode == ENCODER_MOTOR_MODE_CALIBRATION_OPEN_LOOP) {
        g_controller[ENCODER_MOTOR_LEFT].targetCounts = 0;
        g_controller[ENCODER_MOTOR_RIGHT].targetCounts = 0;
        g_controller[ENCODER_MOTOR_LEFT].feedbackCounts = leftFeedback;
        g_controller[ENCODER_MOTOR_RIGHT].feedbackCounts = rightFeedback;
        g_controller[ENCODER_MOTOR_LEFT].integralScaled = 0;
        g_controller[ENCODER_MOTOR_RIGHT].integralScaled = 0;
        output[ENCODER_MOTOR_LEFT] =
            g_controller[ENCODER_MOTOR_LEFT].openLoopPwm;
        output[ENCODER_MOTOR_RIGHT] =
            g_controller[ENCODER_MOTOR_RIGHT].openLoopPwm;
        g_controller[ENCODER_MOTOR_LEFT].pwm = output[ENCODER_MOTOR_LEFT];
        g_controller[ENCODER_MOTOR_RIGHT].pwm = output[ENCODER_MOTOR_RIGHT];
    } else if (g_mode == ENCODER_MOTOR_MODE_CROSS_COUPLED_PWM) {
        int32_t correction = EncoderMotor_CalculateCrossPwmSyncCorrection(
            leftFeedback, rightFeedback,
            g_controller[ENCODER_MOTOR_LEFT].openLoopPwm,
            g_controller[ENCODER_MOTOR_RIGHT].openLoopPwm,
            g_crossPwmSyncGainQ1024, g_crossPwmSyncLimit);

        g_controller[ENCODER_MOTOR_LEFT].targetCounts = 0;
        g_controller[ENCODER_MOTOR_RIGHT].targetCounts = 0;
        g_controller[ENCODER_MOTOR_LEFT].feedbackCounts = leftFeedback;
        g_controller[ENCODER_MOTOR_RIGHT].feedbackCounts = rightFeedback;
        g_controller[ENCODER_MOTOR_LEFT].integralScaled = 0;
        g_controller[ENCODER_MOTOR_RIGHT].integralScaled = 0;
        output[ENCODER_MOTOR_LEFT] = EncoderMotor_ClampPwm(
            (int32_t)g_controller[ENCODER_MOTOR_LEFT].openLoopPwm -
            correction);
        output[ENCODER_MOTOR_RIGHT] = EncoderMotor_ClampPwm(
            (int32_t)g_controller[ENCODER_MOTOR_RIGHT].openLoopPwm +
            correction);
        g_controller[ENCODER_MOTOR_LEFT].pwm = output[ENCODER_MOTOR_LEFT];
        g_controller[ENCODER_MOTOR_RIGHT].pwm = output[ENCODER_MOTOR_RIGHT];
    } else {
        targetCounts[ENCODER_MOTOR_LEFT] =
            g_controller[ENCODER_MOTOR_LEFT].commandCounts;
        targetCounts[ENCODER_MOTOR_RIGHT] =
            g_controller[ENCODER_MOTOR_RIGHT].commandCounts;
        if ((g_mode == ENCODER_MOTOR_MODE_NORMAL) &&
            (targetCounts[ENCODER_MOTOR_LEFT] != 0) &&
            (targetCounts[ENCODER_MOTOR_LEFT] ==
                targetCounts[ENCODER_MOTOR_RIGHT])) {
            straightSyncCorrection =
                EncoderMotor_CalculateStraightSyncCorrection(
                    leftFeedback, rightFeedback,
                    targetCounts[ENCODER_MOTOR_LEFT]);
            targetCounts[ENCODER_MOTOR_LEFT] =
                EncoderMotor_ClampPeriodTarget(
                    targetCounts[ENCODER_MOTOR_LEFT] -
                    straightSyncCorrection);
            targetCounts[ENCODER_MOTOR_RIGHT] =
                EncoderMotor_ClampPeriodTarget(
                    targetCounts[ENCODER_MOTOR_RIGHT] +
                    straightSyncCorrection);
        }
        output[ENCODER_MOTOR_LEFT] = EncoderMotor_UpdateClosedLoop(
            ENCODER_MOTOR_LEFT, targetCounts[ENCODER_MOTOR_LEFT],
            leftFeedback);
        output[ENCODER_MOTOR_RIGHT] = EncoderMotor_UpdateClosedLoop(
            ENCODER_MOTOR_RIGHT, targetCounts[ENCODER_MOTOR_RIGHT],
            rightFeedback);
    }
    EncoderMotor_WritePwm(ENCODER_MOTOR_LEFT, output[ENCODER_MOTOR_LEFT]);
    EncoderMotor_WritePwm(ENCODER_MOTOR_RIGHT, output[ENCODER_MOTOR_RIGHT]);
    ++g_sampleSequence;
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_Stop(void)
{
    uint8_t index;
    uint32_t primask = EncoderMotor_EnterCritical();

    for (index = 0U; index < ENCODER_MOTOR_COUNT; ++index) {
        g_controller[index].targetCps = 0;
        g_controller[index].commandCounts = 0;
        g_controller[index].openLoopPwm = 0;
        g_controller[index].pwm = 0;
        g_controller[index].targetCounts = 0;
        g_controller[index].feedbackCounts = 0;
        g_controller[index].integralScaled = 0;
        g_controller[index].startupActive = 0U;
        g_controller[index].zeroTargetBrakeEnabled = 0U;
        g_intervalCount[index] = 0;
    }
    EncoderMotor_WritePwm(ENCODER_MOTOR_LEFT, 0);
    EncoderMotor_WritePwm(ENCODER_MOTOR_RIGHT, 0);
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_EnterCalibration(void)
{
    uint8_t index;
    uint32_t primask = EncoderMotor_EnterCritical();

    g_mode = ENCODER_MOTOR_MODE_CALIBRATION_OPEN_LOOP;
    for (index = 0U; index < ENCODER_MOTOR_COUNT; ++index) {
        g_controller[index].targetCps = 0;
        g_controller[index].commandCounts = 0;
        g_controller[index].openLoopPwm = 0;
        g_controller[index].targetCounts = 0;
        g_controller[index].pwm = 0;
        g_controller[index].integralScaled = 0;
        g_controller[index].startupActive = 0U;
        g_controller[index].zeroTargetBrakeEnabled = 0U;
        g_intervalCount[index] = 0;
    }
    EncoderMotor_WritePwm(ENCODER_MOTOR_LEFT, 0);
    EncoderMotor_WritePwm(ENCODER_MOTOR_RIGHT, 0);
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_ExitCalibration(void)
{
    uint8_t index;
    uint32_t primask = EncoderMotor_EnterCritical();

    g_mode = ENCODER_MOTOR_MODE_NORMAL;
    for (index = 0U; index < ENCODER_MOTOR_COUNT; ++index) {
        g_controller[index].targetCps = 0;
        g_controller[index].commandCounts = 0;
        g_controller[index].openLoopPwm = 0;
        g_controller[index].targetCounts = 0;
        g_controller[index].pwm = 0;
        g_controller[index].integralScaled = 0;
        g_controller[index].startupActive = 0U;
        g_controller[index].zeroTargetBrakeEnabled = 0U;
        g_intervalCount[index] = 0;
    }
    EncoderMotor_WritePwm(ENCODER_MOTOR_LEFT, 0);
    EncoderMotor_WritePwm(ENCODER_MOTOR_RIGHT, 0);
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_SetOpenLoopPwm(int16_t leftPwm, int16_t rightPwm)
{
    uint32_t primask = EncoderMotor_EnterCritical();

    g_mode = ENCODER_MOTOR_MODE_CALIBRATION_OPEN_LOOP;
    g_controller[ENCODER_MOTOR_LEFT].openLoopPwm =
        EncoderMotor_ClampPwm(leftPwm);
    g_controller[ENCODER_MOTOR_RIGHT].openLoopPwm =
        EncoderMotor_ClampPwm(rightPwm);
    g_controller[ENCODER_MOTOR_LEFT].commandCounts = 0;
    g_controller[ENCODER_MOTOR_RIGHT].commandCounts = 0;
    g_controller[ENCODER_MOTOR_LEFT].targetCounts = 0;
    g_controller[ENCODER_MOTOR_RIGHT].targetCounts = 0;
    g_controller[ENCODER_MOTOR_LEFT].integralScaled = 0;
    g_controller[ENCODER_MOTOR_RIGHT].integralScaled = 0;
    g_controller[ENCODER_MOTOR_LEFT].startupActive = 0U;
    g_controller[ENCODER_MOTOR_RIGHT].startupActive = 0U;
    g_controller[ENCODER_MOTOR_LEFT].zeroTargetBrakeEnabled = 0U;
    g_controller[ENCODER_MOTOR_RIGHT].zeroTargetBrakeEnabled = 0U;
    g_controller[ENCODER_MOTOR_LEFT].pwm =
        g_controller[ENCODER_MOTOR_LEFT].openLoopPwm;
    g_controller[ENCODER_MOTOR_RIGHT].pwm =
        g_controller[ENCODER_MOTOR_RIGHT].openLoopPwm;
    EncoderMotor_WritePwm(ENCODER_MOTOR_LEFT,
        g_controller[ENCODER_MOTOR_LEFT].openLoopPwm);
    EncoderMotor_WritePwm(ENCODER_MOTOR_RIGHT,
        g_controller[ENCODER_MOTOR_RIGHT].openLoopPwm);
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_SetCalibrationTargets(int16_t leftCounts,
    int16_t rightCounts)
{
    int16_t nextLeft = EncoderMotor_ClampPeriodTarget(leftCounts);
    int16_t nextRight = EncoderMotor_ClampPeriodTarget(rightCounts);
    uint32_t primask = EncoderMotor_EnterCritical();

    g_mode = ENCODER_MOTOR_MODE_CALIBRATION_CLOSED_LOOP;
    g_controller[ENCODER_MOTOR_LEFT].integralScaled = 0;
    g_controller[ENCODER_MOTOR_RIGHT].integralScaled = 0;
    EncoderMotor_ArmStartupCompensationLocked(nextLeft, nextRight);
    EncoderMotor_ApplyTargetLocked(ENCODER_MOTOR_LEFT,
        (int16_t)(nextLeft * (int16_t)CHASSIS_SPEED_UNIT_HZ), nextLeft);
    EncoderMotor_ApplyTargetLocked(ENCODER_MOTOR_RIGHT,
        (int16_t)(nextRight * (int16_t)CHASSIS_SPEED_UNIT_HZ), nextRight);
    g_controller[ENCODER_MOTOR_LEFT].openLoopPwm = 0;
    g_controller[ENCODER_MOTOR_RIGHT].openLoopPwm = 0;
    g_controller[ENCODER_MOTOR_LEFT].pwm = 0;
    g_controller[ENCODER_MOTOR_RIGHT].pwm = 0;
    EncoderMotor_WritePwm(ENCODER_MOTOR_LEFT, 0);
    EncoderMotor_WritePwm(ENCODER_MOTOR_RIGHT, 0);
    EncoderMotor_ExitCritical(primask);
}

static int16_t EncoderMotor_ClampPositivePwm(int16_t value)
{
    if (value < 0) {
        return 0;
    }
    return EncoderMotor_ClampPwm(value);
}

static int32_t EncoderMotor_ClampGain(int32_t value)
{
    return (value < 0) ? 0 : value;
}

void EncoderMotor_SetTuning(const EncoderMotorTuning *left,
    const EncoderMotorTuning *right)
{
    uint32_t primask;

    if ((left == 0) || (right == 0)) {
        return;
    }
    primask = EncoderMotor_EnterCritical();
    g_tuning[ENCODER_MOTOR_LEFT] = *left;
    g_tuning[ENCODER_MOTOR_RIGHT] = *right;
    g_tuning[ENCODER_MOTOR_LEFT].startPwm =
        EncoderMotor_ClampPositivePwm(left->startPwm);
    g_tuning[ENCODER_MOTOR_RIGHT].startPwm =
        EncoderMotor_ClampPositivePwm(right->startPwm);
    g_tuning[ENCODER_MOTOR_LEFT].runStartPwm =
        EncoderMotor_ClampPositivePwm(left->runStartPwm);
    g_tuning[ENCODER_MOTOR_RIGHT].runStartPwm =
        EncoderMotor_ClampPositivePwm(right->runStartPwm);
    g_tuning[ENCODER_MOTOR_LEFT].integralLimitPwm =
        EncoderMotor_ClampPositivePwm(left->integralLimitPwm);
    g_tuning[ENCODER_MOTOR_RIGHT].integralLimitPwm =
        EncoderMotor_ClampPositivePwm(right->integralLimitPwm);
    g_tuning[ENCODER_MOTOR_LEFT].ffQ1024 =
        EncoderMotor_ClampGain(left->ffQ1024);
    g_tuning[ENCODER_MOTOR_RIGHT].ffQ1024 =
        EncoderMotor_ClampGain(right->ffQ1024);
    g_tuning[ENCODER_MOTOR_LEFT].kpQ1024 =
        EncoderMotor_ClampGain(left->kpQ1024);
    g_tuning[ENCODER_MOTOR_RIGHT].kpQ1024 =
        EncoderMotor_ClampGain(right->kpQ1024);
    g_tuning[ENCODER_MOTOR_LEFT].kiQ1024 =
        EncoderMotor_ClampGain(left->kiQ1024);
    g_tuning[ENCODER_MOTOR_RIGHT].kiQ1024 =
        EncoderMotor_ClampGain(right->kiQ1024);
    g_controller[ENCODER_MOTOR_LEFT].integralScaled = 0;
    g_controller[ENCODER_MOTOR_RIGHT].integralScaled = 0;
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_ClearIntegral(void)
{
    uint32_t primask = EncoderMotor_EnterCritical();

    g_controller[ENCODER_MOTOR_LEFT].integralScaled = 0;
    g_controller[ENCODER_MOTOR_RIGHT].integralScaled = 0;
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_GetSnapshot(EncoderMotorSnapshot *snapshot)
{
    uint8_t index;
    uint8_t rightEncoderState;
    uint32_t primask;

    if (snapshot == 0) {
        return;
    }
    primask = EncoderMotor_EnterCritical();
    snapshot->mode = g_mode;
    snapshot->sampleSequence = g_sampleSequence;
    snapshot->pwmCompareCounts[ENCODER_MOTOR_LEFT] =
        DL_TimerA_getCaptureCompareValue(TIMA0,
            PIN_CHASSIS_LEFT_PWM_CC_INDEX);
    snapshot->pwmCompareCounts[ENCODER_MOTOR_RIGHT] =
        DL_TimerA_getCaptureCompareValue(TIMA0,
            PIN_CHASSIS_RIGHT_PWM_CC_INDEX);
    snapshot->rightEncoderAInterruptCount =
        g_rightEncoderAInterruptCount;
    snapshot->rightEncoderBInterruptCount =
        g_rightEncoderBInterruptCount;
    rightEncoderState = EncoderMotor_ReadState(ENCODER_MOTOR_RIGHT);
    snapshot->rightEncoderALevel = (uint8_t)((rightEncoderState >> 1U) & 1U);
    snapshot->rightEncoderBLevel = (uint8_t)(rightEncoderState & 1U);
    for (index = 0U; index < ENCODER_MOTOR_COUNT; ++index) {
        snapshot->targetCounts[index] = g_controller[index].targetCounts;
        snapshot->feedbackCounts[index] =
            g_controller[index].feedbackCounts;
        snapshot->outputPwm[index] = g_controller[index].pwm;
        snapshot->integralOutputPwm[index] = (int32_t)
            (g_controller[index].integralScaled / CHASSIS_Q1024_SCALE);
        snapshot->startupActive[index] =
            g_controller[index].startupActive;
        snapshot->tuning[index] = g_tuning[index];
    }
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_HandleGPIOInterrupt(void)
{
    uint32_t pendingLeftA = DL_GPIO_getEnabledInterruptStatus(
        PIN_CHASSIS_LEFT_ENCODER_A_PORT, PIN_CHASSIS_LEFT_ENCODER_A);
    uint32_t pendingLeftB = DL_GPIO_getEnabledInterruptStatus(
        PIN_CHASSIS_LEFT_ENCODER_B_PORT, PIN_CHASSIS_LEFT_ENCODER_B);
    uint32_t pendingRightA = DL_GPIO_getEnabledInterruptStatus(
        PIN_CHASSIS_RIGHT_ENCODER_A_PORT, PIN_CHASSIS_RIGHT_ENCODER_A);
    uint32_t pendingRightB = DL_GPIO_getEnabledInterruptStatus(
        PIN_CHASSIS_RIGHT_ENCODER_B_PORT, PIN_CHASSIS_RIGHT_ENCODER_B);

    if (pendingRightA != 0U) {
        ++g_rightEncoderAInterruptCount;
    }
    if (pendingRightB != 0U) {
        ++g_rightEncoderBInterruptCount;
    }
    if ((pendingLeftA != 0U) || (pendingLeftB != 0U)) {
        EncoderMotor_UpdateEncoder(ENCODER_MOTOR_LEFT);
    }
    if ((pendingRightA != 0U) || (pendingRightB != 0U)) {
        EncoderMotor_UpdateEncoder(ENCODER_MOTOR_RIGHT);
    }
    if (pendingLeftA != 0U) {
        DL_GPIO_clearInterruptStatus(PIN_CHASSIS_LEFT_ENCODER_A_PORT,
            pendingLeftA);
    }
    if (pendingLeftB != 0U) {
        DL_GPIO_clearInterruptStatus(PIN_CHASSIS_LEFT_ENCODER_B_PORT,
            pendingLeftB);
    }
    if (pendingRightA != 0U) {
        DL_GPIO_clearInterruptStatus(PIN_CHASSIS_RIGHT_ENCODER_A_PORT,
            pendingRightA);
    }
    if (pendingRightB != 0U) {
        DL_GPIO_clearInterruptStatus(PIN_CHASSIS_RIGHT_ENCODER_B_PORT,
            pendingRightB);
    }
}

int16_t EncoderMotor_GetTarget(uint8_t motorIndex)
{
    return (motorIndex < ENCODER_MOTOR_COUNT) ?
        g_controller[motorIndex].targetCps : 0;
}

int16_t EncoderMotor_GetPwm(uint8_t motorIndex)
{
    return (motorIndex < ENCODER_MOTOR_COUNT) ?
        g_controller[motorIndex].pwm : 0;
}

int32_t EncoderMotor_GetTotalCount(uint8_t motorIndex)
{
    int32_t count;
    uint32_t primask;

    if (motorIndex >= ENCODER_MOTOR_COUNT) {
        return 0;
    }
    primask = EncoderMotor_EnterCritical();
    count = g_totalCount[motorIndex];
    EncoderMotor_ExitCritical(primask);
    return count;
}

void EncoderMotor_ResetTotalCount(uint8_t motorIndex)
{
    uint32_t primask;

    if (motorIndex >= ENCODER_MOTOR_COUNT) {
        return;
    }
    primask = EncoderMotor_EnterCritical();
    g_totalCount[motorIndex] = 0;
    g_intervalCount[motorIndex] = 0;
    EncoderMotor_ExitCritical(primask);
}

void EncoderMotor_ResetAllCounts(void)
{
    EncoderMotor_ResetTotalCount(ENCODER_MOTOR_LEFT);
    EncoderMotor_ResetTotalCount(ENCODER_MOTOR_RIGHT);
}
