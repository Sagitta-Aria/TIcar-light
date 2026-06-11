#include "stepper_pulse.h"

#include "board_config.h"
#include "pin_map.h"

#if (CAR_STEPPER_PULSE_HIGH_TICKS == 0U)
#error "CAR_STEPPER_PULSE_HIGH_TICKS must be greater than 0"
#endif

#if (CAR_STEPPER_RAMP_PERIOD_MS == 0U)
#error "CAR_STEPPER_RAMP_PERIOD_MS must be greater than 0"
#endif

#if (CAR_STEPPER_ACCEL_STEP_SPS == 0U)
#error "CAR_STEPPER_ACCEL_STEP_SPS must be greater than 0"
#endif

#if (CAR_STEPPER_DECEL_STEP_SPS == 0U)
#error "CAR_STEPPER_DECEL_STEP_SPS must be greater than 0"
#endif

#define STEPPER_RAMP_INTERVAL_TICKS \
    ((STEPPER_TIMER_TICK_HZ * CAR_STEPPER_RAMP_PERIOD_MS) / 1000U)

#if (STEPPER_RAMP_INTERVAL_TICKS == 0U)
#error "STEPPER_RAMP_INTERVAL_TICKS must be greater than 0"
#endif

typedef struct {
    GPIO_Regs *stepPort;
    uint32_t stepPin;
    volatile uint32_t stepRateHz;
    volatile uint32_t targetRateHz;
    volatile uint32_t accumulator;
    volatile uint32_t rampTicks;
    volatile uint8_t highTicksLeft;
    volatile int8_t directionSign;
    volatile int32_t stepCount;
} StepperPulseChannel;

typedef struct {
    uint32_t gpioaSetMask;
    uint32_t gpioaClearMask;
    uint32_t gpiobSetMask;
    uint32_t gpiobClearMask;
} StepperPulseGpioBatch;

static StepperPulseChannel g_stepperPulse[MOTOR_COUNT] = {
    {
        PIN_STEPPER_CHASSIS_LEFT_STEP_PORT,
        PIN_STEPPER_CHASSIS_LEFT_STEP,
        0U, 0U, 0U, 0U, 0U, 1, 0
    },
    {
        PIN_STEPPER_CHASSIS_RIGHT_STEP_PORT,
        PIN_STEPPER_CHASSIS_RIGHT_STEP,
        0U, 0U, 0U, 0U, 0U, 1, 0
    },
    {
        PIN_STEPPER_GIMBAL_1_STEP_PORT,
        PIN_STEPPER_GIMBAL_1_STEP,
        0U, 0U, 0U, 0U, 0U, 1, 0
    },
    {
        PIN_STEPPER_GIMBAL_2_STEP_PORT,
        PIN_STEPPER_GIMBAL_2_STEP,
        0U, 0U, 0U, 0U, 0U, 1, 0
    }
};

static uint8_t StepperPulse_IsValid(MotorId motor)
{
    return ((uint32_t)motor < (uint32_t)MOTOR_COUNT) ? 1U : 0U;
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
    channel->targetRateHz = 0U;
    channel->accumulator = 0U;
    channel->rampTicks = 0U;
    channel->highTicksLeft = 0U;
    channel->directionSign = 1;
    DL_GPIO_clearPins(channel->stepPort, channel->stepPin);
}

/*
 * 作用：把实际 STEP 频率按固定斜坡靠近目标频率。
 * 使用场景：TIMG0 ISR 内部调用，削弱速度突变带来的顿挫。
 * 说明：0 速命令由 SetTarget 立即停车，保留异常停车和死区停轴的响应速度。
 */
static void StepperPulse_UpdateRampOne(StepperPulseChannel *channel)
{
    uint32_t step;

    if (channel->stepRateHz == channel->targetRateHz) {
        channel->rampTicks = 0U;
        return;
    }

    ++channel->rampTicks;
    if (channel->rampTicks < STEPPER_RAMP_INTERVAL_TICKS) {
        return;
    }
    channel->rampTicks = 0U;

    if (channel->stepRateHz < channel->targetRateHz) {
        step = channel->targetRateHz - channel->stepRateHz;
        if (step > CAR_STEPPER_ACCEL_STEP_SPS) {
            step = CAR_STEPPER_ACCEL_STEP_SPS;
        }
        channel->stepRateHz += step;
    } else {
        step = channel->stepRateHz - channel->targetRateHz;
        if (step > CAR_STEPPER_DECEL_STEP_SPS) {
            step = CAR_STEPPER_DECEL_STEP_SPS;
        }
        channel->stepRateHz -= step;
    }
}

/* 作用：把单个 STEP 引脚动作加入本次定时器 tick 的批量 GPIO 写入。 */
static void StepperPulse_AddPinAction(StepperPulseGpioBatch *batch,
    GPIO_Regs *port, uint32_t pin, uint8_t setHigh)
{
    if (port == GPIOA) {
        if (setHigh != 0U) {
            batch->gpioaSetMask |= pin;
        } else {
            batch->gpioaClearMask |= pin;
        }
    } else if (port == GPIOB) {
        if (setHigh != 0U) {
            batch->gpiobSetMask |= pin;
        } else {
            batch->gpiobClearMask |= pin;
        }
    } else if (setHigh != 0U) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

/* 作用：把同一个定时器 tick 内的 STEP 电平变化一次性写到 GPIO。 */
static void StepperPulse_ApplyBatch(const StepperPulseGpioBatch *batch)
{
    if (batch->gpioaClearMask != 0U) {
        DL_GPIO_clearPins(GPIOA, batch->gpioaClearMask);
    }
    if (batch->gpiobClearMask != 0U) {
        DL_GPIO_clearPins(GPIOB, batch->gpiobClearMask);
    }
    if (batch->gpioaSetMask != 0U) {
        DL_GPIO_setPins(GPIOA, batch->gpioaSetMask);
    }
    if (batch->gpiobSetMask != 0U) {
        DL_GPIO_setPins(GPIOB, batch->gpiobSetMask);
    }
}

/*
 * 作用：按一个定时器 tick 刷新单个电机 STEP 输出。
 * 使用场景：StepperPulse_HandleTimerInterrupt 对四个电机轮询调用。
 */
static void StepperPulse_TickOne(StepperPulseChannel *channel,
    StepperPulseGpioBatch *batch)
{
    StepperPulse_UpdateRampOne(channel);

    if (channel->highTicksLeft > 0U) {
        --channel->highTicksLeft;
        if (channel->highTicksLeft == 0U) {
            StepperPulse_AddPinAction(batch, channel->stepPort,
                channel->stepPin, 0U);
        }
    }

    if (channel->stepRateHz == 0U) {
        channel->accumulator = 0U;
        StepperPulse_AddPinAction(batch, channel->stepPort,
            channel->stepPin, 0U);
        return;
    }

    channel->accumulator += channel->stepRateHz;
    if (channel->accumulator >= STEPPER_TIMER_TICK_HZ) {
        channel->accumulator -= STEPPER_TIMER_TICK_HZ;
        StepperPulse_AddPinAction(batch, channel->stepPort,
            channel->stepPin, 1U);
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
    uint16_t speedSps)
{
    StepperPulseChannel *channel;
    uint32_t primask;

    if (!StepperPulse_IsValid(motor)) {
        return;
    }

    channel = &g_stepperPulse[(uint32_t)motor];
    primask = StepperPulse_EnterCritical();
    channel->directionSign = (directionSign < 0) ? -1 : 1;
    if (speedSps == 0U) {
        channel->stepRateHz = 0U;
        channel->targetRateHz = 0U;
        channel->accumulator = 0U;
        channel->rampTicks = 0U;
        channel->highTicksLeft = 0U;
        DL_GPIO_clearPins(channel->stepPort, channel->stepPin);
    } else {
        channel->targetRateHz = speedSps;
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
    StepperPulseGpioBatch batch = {0U, 0U, 0U, 0U};

    switch (DL_TimerG_getPendingInterrupt(STEPPER_TIMER_INST)) {
        case DL_TIMER_IIDX_ZERO:
            for (i = 0U; i < (uint32_t)MOTOR_COUNT; ++i) {
                StepperPulse_TickOne(&g_stepperPulse[i], &batch);
            }
            StepperPulse_ApplyBatch(&batch);
            break;
        default:
            break;
    }
}
