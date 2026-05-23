#include "stepper_pulse.h"

#include "board_config.h"
#include "pin_map.h"

#if (CAR_STEPPER_COMMAND_TO_HZ_DIVISOR == 0U)
#error "CAR_STEPPER_COMMAND_TO_HZ_DIVISOR must be greater than 0"
#endif

#if (CAR_STEPPER_PULSE_HIGH_TICKS == 0U)
#error "CAR_STEPPER_PULSE_HIGH_TICKS must be greater than 0"
#endif

typedef struct {
    GPIO_Regs *stepPort;
    uint32_t stepPin;
    volatile uint32_t stepRateHz;
    volatile uint32_t accumulator;
    volatile uint8_t highTicksLeft;
    volatile int8_t directionSign;
    volatile int32_t stepCount;
} StepperPulseChannel;

static StepperPulseChannel g_stepperPulse[MOTOR_COUNT] = {
    {
        PIN_STEPPER_CHASSIS_LEFT_STEP_PORT,
        PIN_STEPPER_CHASSIS_LEFT_STEP,
        0U, 0U, 0U, 1, 0
    },
    {
        PIN_STEPPER_CHASSIS_RIGHT_STEP_PORT,
        PIN_STEPPER_CHASSIS_RIGHT_STEP,
        0U, 0U, 0U, 1, 0
    },
    {
        PIN_STEPPER_GIMBAL_1_STEP_PORT,
        PIN_STEPPER_GIMBAL_1_STEP,
        0U, 0U, 0U, 1, 0
    },
    {
        PIN_STEPPER_GIMBAL_2_STEP_PORT,
        PIN_STEPPER_GIMBAL_2_STEP,
        0U, 0U, 0U, 1, 0
    }
};

static uint8_t StepperPulse_IsValid(MotorId motor)
{
    return ((uint32_t)motor < (uint32_t)MOTOR_COUNT) ? 1U : 0U;
}

/*
 * 作用：把工程速度命令换算成 STEP 频率。
 * 使用场景：主循环设置目标速度时调用，中断里只使用换算后的 Hz。
 * 说明：ccs1.2 保持上一版的低速手感，4000 命令约等于 400 step/s。
 */
static uint32_t StepperPulse_CommandToHz(uint16_t command)
{
    uint32_t hz;

    if (command == 0U) {
        return 0U;
    }

    hz = ((uint32_t)command + (CAR_STEPPER_COMMAND_TO_HZ_DIVISOR / 2U)) /
        CAR_STEPPER_COMMAND_TO_HZ_DIVISOR;
    return (hz == 0U) ? 1U : hz;
}

/*
 * 作用：极短临界区保护主循环和 TIMG0 ISR 共享的调度状态。
 * 使用场景：设置速度、停车、初始化。
 */
static uint32_t StepperPulse_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void StepperPulse_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static void StepperPulse_ResetOne(StepperPulseChannel *channel)
{
    channel->stepRateHz = 0U;
    channel->accumulator = 0U;
    channel->highTicksLeft = 0U;
    channel->directionSign = 1;
    DL_GPIO_clearPins(channel->stepPort, channel->stepPin);
}

/*
 * 作用：按一个定时器 tick 刷新单个电机 STEP 输出。
 * 使用场景：StepperPulse_HandleTimerInterrupt 对四个电机轮询调用。
 */
static void StepperPulse_TickOne(StepperPulseChannel *channel)
{
    if (channel->highTicksLeft > 0U) {
        --channel->highTicksLeft;
        if (channel->highTicksLeft == 0U) {
            DL_GPIO_clearPins(channel->stepPort, channel->stepPin);
        }
        return;
    }

    if (channel->stepRateHz == 0U) {
        channel->accumulator = 0U;
        DL_GPIO_clearPins(channel->stepPort, channel->stepPin);
        return;
    }

    channel->accumulator += channel->stepRateHz;
    if (channel->accumulator >= STEPPER_TIMER_TICK_HZ) {
        channel->accumulator -= STEPPER_TIMER_TICK_HZ;
        DL_GPIO_setPins(channel->stepPort, channel->stepPin);
        channel->highTicksLeft = CAR_STEPPER_PULSE_HIGH_TICKS;
        channel->stepCount += (int32_t)channel->directionSign;
    }
}

void StepperPulse_Init(void)
{
    StepperPulse_StopAll();
    NVIC_ClearPendingIRQ(STEPPER_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(STEPPER_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(STEPPER_TIMER_INST);
}

void StepperPulse_SetTarget(MotorId motor, int8_t directionSign,
    uint16_t command)
{
    StepperPulseChannel *channel;
    uint32_t primask;

    if (!StepperPulse_IsValid(motor)) {
        return;
    }

    channel = &g_stepperPulse[(uint32_t)motor];
    primask = StepperPulse_EnterCritical();
    channel->stepRateHz = StepperPulse_CommandToHz(command);
    channel->directionSign = (directionSign < 0) ? -1 : 1;
    if (command == 0U) {
        channel->accumulator = 0U;
        channel->highTicksLeft = 0U;
        DL_GPIO_clearPins(channel->stepPort, channel->stepPin);
    }
    StepperPulse_ExitCritical(primask);
}

void StepperPulse_StopAll(void)
{
    uint32_t i;
    uint32_t primask = StepperPulse_EnterCritical();

    for (i = 0U; i < (uint32_t)MOTOR_COUNT; ++i) {
        StepperPulse_ResetOne(&g_stepperPulse[i]);
    }
    StepperPulse_ExitCritical(primask);
}

int32_t StepperPulse_GetStepCount(MotorId motor)
{
    int32_t count;
    uint32_t primask;

    if (!StepperPulse_IsValid(motor)) {
        return 0;
    }

    primask = StepperPulse_EnterCritical();
    count = g_stepperPulse[(uint32_t)motor].stepCount;
    StepperPulse_ExitCritical(primask);
    return count;
}

void StepperPulse_ResetStepCount(MotorId motor)
{
    uint32_t primask;

    if (!StepperPulse_IsValid(motor)) {
        return;
    }

    primask = StepperPulse_EnterCritical();
    g_stepperPulse[(uint32_t)motor].stepCount = 0;
    StepperPulse_ExitCritical(primask);
}

void StepperPulse_ResetAllStepCounts(void)
{
    uint32_t i;
    uint32_t primask = StepperPulse_EnterCritical();

    for (i = 0U; i < (uint32_t)MOTOR_COUNT; ++i) {
        g_stepperPulse[i].stepCount = 0;
    }
    StepperPulse_ExitCritical(primask);
}

void StepperPulse_HandleTimerInterrupt(void)
{
    uint32_t i;

    switch (DL_TimerG_getPendingInterrupt(STEPPER_TIMER_INST)) {
        case DL_TIMER_IIDX_ZERO:
            for (i = 0U; i < (uint32_t)MOTOR_COUNT; ++i) {
                StepperPulse_TickOne(&g_stepperPulse[i]);
            }
            break;
        default:
            break;
    }
}
