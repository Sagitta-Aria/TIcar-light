#include "stepper_pin_test.h"

#include "board_config.h"
#include "delay.h"
#include "ti_msp_dl_config.h"

typedef enum {
    STEPPER_PIN_TEST_LR_FORWARD = 0,
    STEPPER_PIN_TEST_LR_REVERSE,
    STEPPER_PIN_TEST_UD_FORWARD,
    STEPPER_PIN_TEST_UD_REVERSE,
    STEPPER_PIN_TEST_PAUSE
} StepperPinTestStage;

typedef struct {
    GPIO_Regs *stepPort;
    uint32_t stepPin;
    GPIO_Regs *dirPort;
    uint32_t dirPin;
} StepperPinTestAxis;

typedef struct {
    StepperPinTestStage stage;
    StepperPinTestStage nextStage;
    uint32_t stepCount;
    uint32_t pauseTicks;
} StepperPinTestState;

static const StepperPinTestAxis g_stepperPinTestAxis[2] = {
    {
        STEPPER_GIMBAL_1_STEP_PORT,
        STEPPER_GIMBAL_1_STEP_PIN,
        STEPPER_GIMBAL_1_DIR_PORT,
        STEPPER_GIMBAL_1_DIR_PIN
    },
    {
        STEPPER_GIMBAL_2_STEP_PORT,
        STEPPER_GIMBAL_2_STEP_PIN,
        STEPPER_GIMBAL_2_DIR_PORT,
        STEPPER_GIMBAL_2_DIR_PIN
    }
};

static StepperPinTestState g_stepperPinTest;

/*
 * 作用：输出一个满足驱动器识别的短 STEP 高脉冲。
 * 说明：32MHz 下 320 个 cycle 约 10us，足够常见闭环步进驱动器识别。
 */
static void StepperPinTest_Pulse(const StepperPinTestAxis *axis)
{
    DL_GPIO_setPins(axis->stepPort, axis->stepPin);
    delay_cycles(320U);
    DL_GPIO_clearPins(axis->stepPort, axis->stepPin);
}

static void StepperPinTest_SetDir(const StepperPinTestAxis *axis,
    uint8_t reverse)
{
    if (reverse != 0U) {
        DL_GPIO_setPins(axis->dirPort, axis->dirPin);
    } else {
        DL_GPIO_clearPins(axis->dirPort, axis->dirPin);
    }
}

static const StepperPinTestAxis *StepperPinTest_GetAxis(
    StepperPinTestStage stage)
{
    if ((stage == STEPPER_PIN_TEST_UD_FORWARD) ||
        (stage == STEPPER_PIN_TEST_UD_REVERSE)) {
        return &g_stepperPinTestAxis[1];
    }
    return &g_stepperPinTestAxis[0];
}

static uint8_t StepperPinTest_IsReverse(StepperPinTestStage stage)
{
    return ((stage == STEPPER_PIN_TEST_LR_REVERSE) ||
        (stage == STEPPER_PIN_TEST_UD_REVERSE)) ? 1U : 0U;
}

static StepperPinTestStage StepperPinTest_GetNextMove(
    StepperPinTestStage stage)
{
    switch (stage) {
    case STEPPER_PIN_TEST_LR_FORWARD:
        return STEPPER_PIN_TEST_LR_REVERSE;
    case STEPPER_PIN_TEST_LR_REVERSE:
        return STEPPER_PIN_TEST_UD_FORWARD;
    case STEPPER_PIN_TEST_UD_FORWARD:
        return STEPPER_PIN_TEST_UD_REVERSE;
    case STEPPER_PIN_TEST_UD_REVERSE:
    default:
        return STEPPER_PIN_TEST_LR_FORWARD;
    }
}

static void StepperPinTest_EnterMove(StepperPinTestStage stage)
{
    const StepperPinTestAxis *axis = StepperPinTest_GetAxis(stage);

    g_stepperPinTest.stage = stage;
    g_stepperPinTest.stepCount = 0U;
    StepperPinTest_SetDir(axis, StepperPinTest_IsReverse(stage));
}

static void StepperPinTest_EnterPause(StepperPinTestStage nextStage)
{
    g_stepperPinTest.stage = STEPPER_PIN_TEST_PAUSE;
    g_stepperPinTest.nextStage = nextStage;
    g_stepperPinTest.pauseTicks = 0U;
}

void StepperPinTest_InitPins(void)
{
    DL_GPIO_initDigitalOutput(STEPPER_GIMBAL_1_STEP_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_GIMBAL_1_DIR_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_GIMBAL_2_STEP_IOMUX);
    DL_GPIO_initDigitalOutput(STEPPER_GIMBAL_2_DIR_IOMUX);

    DL_GPIO_clearPins(STEPPER_GIMBAL_1_STEP_PORT,
        STEPPER_GIMBAL_1_STEP_PIN);
    DL_GPIO_clearPins(STEPPER_GIMBAL_1_DIR_PORT,
        STEPPER_GIMBAL_1_DIR_PIN);
    DL_GPIO_clearPins(STEPPER_GIMBAL_2_STEP_PORT,
        STEPPER_GIMBAL_2_STEP_PIN);
    DL_GPIO_clearPins(STEPPER_GIMBAL_2_DIR_PORT,
        STEPPER_GIMBAL_2_DIR_PIN);

    DL_GPIO_enableOutput(STEPPER_GIMBAL_1_STEP_PORT,
        STEPPER_GIMBAL_1_STEP_PIN);
    DL_GPIO_enableOutput(STEPPER_GIMBAL_1_DIR_PORT,
        STEPPER_GIMBAL_1_DIR_PIN);
    DL_GPIO_enableOutput(STEPPER_GIMBAL_2_STEP_PORT,
        STEPPER_GIMBAL_2_STEP_PIN);
    DL_GPIO_enableOutput(STEPPER_GIMBAL_2_DIR_PORT,
        STEPPER_GIMBAL_2_DIR_PIN);
}

void StepperPinTest_Start(void)
{
    g_stepperPinTest.nextStage = STEPPER_PIN_TEST_LR_FORWARD;
    g_stepperPinTest.pauseTicks = 0U;
    StepperPinTest_EnterMove(STEPPER_PIN_TEST_LR_FORWARD);
}

void StepperPinTest_Task(void)
{
    const StepperPinTestAxis *axis;

    if (g_stepperPinTest.stage == STEPPER_PIN_TEST_PAUSE) {
        ++g_stepperPinTest.pauseTicks;
        delay_ms(CAR_GIMBAL_PIN_TEST_PERIOD_MS);
        if (g_stepperPinTest.pauseTicks >= CAR_GIMBAL_PIN_TEST_PAUSE_TICKS) {
            StepperPinTest_EnterMove(g_stepperPinTest.nextStage);
        }
        return;
    }

    axis = StepperPinTest_GetAxis(g_stepperPinTest.stage);
    StepperPinTest_Pulse(axis);
    ++g_stepperPinTest.stepCount;
    delay_ms(CAR_GIMBAL_PIN_TEST_PERIOD_MS);

    if (g_stepperPinTest.stepCount >= CAR_GIMBAL_PIN_TEST_STEPS_PER_MOVE) {
        StepperPinTest_EnterPause(
            StepperPinTest_GetNextMove(g_stepperPinTest.stage));
    }
}
