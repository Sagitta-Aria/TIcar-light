#include "speed_control.h"

#include "board_config.h"
#include "encoder.h"
#include "motor.h"

typedef struct {
    int16_t targetCommand;
    int16_t currentCommand;
    int32_t lastCount;
    int32_t integral;
    int16_t actualTicks;
    int16_t outputCommand;
    int8_t encoderSign;
} SpeedControlWheel;

static SpeedControlWheel g_leftWheel;
static SpeedControlWheel g_rightWheel;
static uint16_t g_speedControlTicks;

/*
 * 作用：限制有符号数范围。
 * 使用场景：限制目标命令、积分和最终输出命令。
 */
static int32_t SpeedControl_Clamp32(int32_t value, int32_t minValue,
    int32_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

/*
 * 作用：求 int16_t 绝对值。
 * 使用场景：处理有符号速度命令。
 */
static uint16_t SpeedControl_Abs16(int16_t value)
{
    return (value < 0) ? (uint16_t)(-value) : (uint16_t)value;
}

/*
 * 作用：返回数值方向，正数为 1，负数为 -1，零为 0。
 * 使用场景：目标方向变化时清积分，防止反向拖拽。
 */
static int8_t SpeedControl_Sign16(int16_t value)
{
    if (value > 0) {
        return 1;
    }
    if (value < 0) {
        return -1;
    }
    return 0;
}

/*
 * 作用：把目标命令限制在电机命令范围内。
 * 使用场景：外层给速度命令时先做保护。
 */
static int16_t SpeedControl_ClampCommand(int16_t command)
{
    return (int16_t)SpeedControl_Clamp32(command,
        -(int32_t)CAR_MOTOR_COMMAND_MAX,
        (int32_t)CAR_MOTOR_COMMAND_MAX);
}

/*
 * 作用：让当前目标按固定步进靠近新目标，避免速度突跳。
 * 使用场景：速度闭环每个控制周期更新目标速度。
 */
static int16_t SpeedControl_StepToward(int16_t current, int16_t target)
{
    int32_t delta = (int32_t)target - (int32_t)current;

    if (delta > (int32_t)CAR_SPEED_TARGET_STEP) {
        delta = (int32_t)CAR_SPEED_TARGET_STEP;
    } else if (delta < -(int32_t)CAR_SPEED_TARGET_STEP) {
        delta = -(int32_t)CAR_SPEED_TARGET_STEP;
    }

    return (int16_t)((int32_t)current + delta);
}

/*
 * 作用：把上层速度命令换算成一个控制周期内的目标编码器增量。
 * 使用场景：PI 环计算速度误差。
 * 说明：命令值最终会映射到底盘 STEP 脉冲节奏。
 */
static int16_t SpeedControl_CommandToTargetTicks(int16_t command)
{
    int32_t targetTicks =
        ((int32_t)command * (int32_t)CAR_SPEED_MAX_TARGET_TICKS) /
        (int32_t)CAR_MOTOR_COMMAND_MAX;

    if ((targetTicks == 0) && (command != 0)) {
        targetTicks = (command > 0) ? 1 : -1;
    }

    return (int16_t)targetTicks;
}

/*
 * 作用：更新某个轮子的目标命令。
 * 使用场景：Motion_SetChassisCommand 写入新的左右轮目标。
 */
static void SpeedControl_SetWheelTarget(SpeedControlWheel *wheel,
    int16_t command)
{
    command = SpeedControl_ClampCommand(command);
    if (SpeedControl_Sign16(wheel->targetCommand) !=
        SpeedControl_Sign16(command)) {
        wheel->integral = 0;
    }
    wheel->targetCommand = command;
}

/*
 * 作用：停止单个轮子的闭环状态。
 * 使用场景：停车、状态机退出循迹、丢线保护。
 */
static void SpeedControl_StopWheel(SpeedControlWheel *wheel, int32_t count)
{
    wheel->targetCommand = 0;
    wheel->currentCommand = 0;
    wheel->lastCount = count;
    wheel->integral = 0;
    wheel->actualTicks = 0;
    wheel->outputCommand = 0;
}

/*
 * 作用：执行一个轮子的 PI 速度控制。
 * 使用场景：SpeedControl_Task 到达控制周期后调用。
 */
static int16_t SpeedControl_UpdateWheel(SpeedControlWheel *wheel,
    int32_t currentCount)
{
    int32_t rawDelta = currentCount - wheel->lastCount;
    int16_t targetTicks;
    int32_t error;
    int32_t output;

    wheel->lastCount = currentCount;
    rawDelta *= (int32_t)wheel->encoderSign;
    rawDelta = SpeedControl_Clamp32(rawDelta, -32768L, 32767L);
    wheel->actualTicks = (int16_t)rawDelta;

    wheel->currentCommand = SpeedControl_StepToward(
        wheel->currentCommand, wheel->targetCommand);

    if ((wheel->targetCommand == 0) && (wheel->currentCommand == 0)) {
        wheel->integral = 0;
        wheel->outputCommand = 0;
        return 0;
    }

    targetTicks = SpeedControl_CommandToTargetTicks(wheel->currentCommand);
    error = (int32_t)targetTicks - (int32_t)wheel->actualTicks;
    wheel->integral += error;
    wheel->integral = SpeedControl_Clamp32(wheel->integral,
        -(int32_t)CAR_SPEED_INTEGRAL_LIMIT,
        (int32_t)CAR_SPEED_INTEGRAL_LIMIT);

    output = (int32_t)wheel->currentCommand +
        ((int32_t)CAR_SPEED_KP * error) +
        ((int32_t)CAR_SPEED_KI * wheel->integral);

    if ((wheel->currentCommand > 0) && (output < 0)) {
        output = 0;
    } else if ((wheel->currentCommand < 0) && (output > 0)) {
        output = 0;
    }

    output = SpeedControl_Clamp32(output,
        -(int32_t)CAR_MOTOR_COMMAND_MAX,
        (int32_t)CAR_MOTOR_COMMAND_MAX);

    if ((wheel->currentCommand != 0) && (output != 0) &&
        (SpeedControl_Abs16((int16_t)output) < CAR_SPEED_MIN_ACTIVE_COMMAND)) {
        output = (wheel->currentCommand > 0) ?
            (int32_t)CAR_SPEED_MIN_ACTIVE_COMMAND :
            -(int32_t)CAR_SPEED_MIN_ACTIVE_COMMAND;
    }

    wheel->outputCommand = (int16_t)output;
    return wheel->outputCommand;
}

void SpeedControl_Init(void)
{
    g_leftWheel.encoderSign = CAR_SPEED_LEFT_ENCODER_SIGN;
    g_rightWheel.encoderSign = CAR_SPEED_RIGHT_ENCODER_SIGN;
    SpeedControl_StopWheel(&g_leftWheel, Encoder_GetLeft());
    SpeedControl_StopWheel(&g_rightWheel, Encoder_GetRight());
    g_speedControlTicks = 0U;
}

void SpeedControl_SetTarget(int16_t left, int16_t right)
{
    SpeedControl_SetWheelTarget(&g_leftWheel, left);
    SpeedControl_SetWheelTarget(&g_rightWheel, right);
}

void SpeedControl_Stop(void)
{
    SpeedControl_StopWheel(&g_leftWheel, Encoder_GetLeft());
    SpeedControl_StopWheel(&g_rightWheel, Encoder_GetRight());
    g_speedControlTicks = 0U;
    Motor_Stop();
}

void SpeedControl_Task(void)
{
#if CAR_ENABLE_SPEED_CONTROL
    int16_t leftOutput;
    int16_t rightOutput;

    ++g_speedControlTicks;
    if (g_speedControlTicks < CAR_SPEED_CONTROL_PERIOD_TICKS) {
        return;
    }
    g_speedControlTicks = 0U;

    leftOutput = SpeedControl_UpdateWheel(&g_leftWheel, Encoder_GetLeft());
    rightOutput = SpeedControl_UpdateWheel(&g_rightWheel, Encoder_GetRight());

    if ((leftOutput == 0) && (rightOutput == 0)) {
        Motor_Stop();
    } else {
        Motor_SetChassisCommand(leftOutput, rightOutput);
    }
#endif
}

int16_t SpeedControl_GetLeftActual(void)
{
    return g_leftWheel.actualTicks;
}

int16_t SpeedControl_GetRightActual(void)
{
    return g_rightWheel.actualTicks;
}

int16_t SpeedControl_GetLeftOutput(void)
{
    return g_leftWheel.outputCommand;
}

int16_t SpeedControl_GetRightOutput(void)
{
    return g_rightWheel.outputCommand;
}
