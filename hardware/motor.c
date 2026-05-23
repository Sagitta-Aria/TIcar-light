#include "motor.h"

#include "board_config.h"
#include "pin_map.h"

typedef struct {
    GPIO_Regs *stepPort;
    uint32_t stepPin;
    GPIO_Regs *dirPort;
    uint32_t dirPin;
    uint16_t command;
    uint16_t accumulator;
    int8_t directionSign;
    int32_t stepCount;
} MotorStepper;

static MotorStepper g_motors[MOTOR_COUNT] = {
    {
        PIN_STEPPER_CHASSIS_LEFT_STEP_PORT,
        PIN_STEPPER_CHASSIS_LEFT_STEP,
        PIN_STEPPER_CHASSIS_LEFT_DIR_PORT,
        PIN_STEPPER_CHASSIS_LEFT_DIR,
        0U, 0U, 1, 0
    },
    {
        PIN_STEPPER_CHASSIS_RIGHT_STEP_PORT,
        PIN_STEPPER_CHASSIS_RIGHT_STEP,
        PIN_STEPPER_CHASSIS_RIGHT_DIR_PORT,
        PIN_STEPPER_CHASSIS_RIGHT_DIR,
        0U, 0U, 1, 0
    },
    {
        PIN_STEPPER_GIMBAL_1_STEP_PORT,
        PIN_STEPPER_GIMBAL_1_STEP,
        PIN_STEPPER_GIMBAL_1_DIR_PORT,
        PIN_STEPPER_GIMBAL_1_DIR,
        0U, 0U, 1, 0
    },
    {
        PIN_STEPPER_GIMBAL_2_STEP_PORT,
        PIN_STEPPER_GIMBAL_2_STEP,
        PIN_STEPPER_GIMBAL_2_DIR_PORT,
        PIN_STEPPER_GIMBAL_2_DIR,
        0U, 0U, 1, 0
    }
};

/*
 * 作用：把上层速度命令限制到步进调度可接受范围。
 * 使用场景：Motor_Set 接收上层速度命令后先做保护。
 */
static uint16_t Motor_ClampCommand(uint16_t command)
{
    return (command > CAR_MOTOR_COMMAND_MAX) ?
        CAR_MOTOR_COMMAND_MAX : command;
}

/*
 * 作用：把 int16_t 速度命令转成正向幅值。
 * 使用场景：Motor_SetChassisCommand 接收左右底盘有符号命令。
 */
static uint16_t Motor_CommandFromSigned(int16_t speed)
{
    int32_t value = (int32_t)speed;

    if (value < 0) {
        value = -value;
    }
    if (value > (int32_t)CAR_MOTOR_COMMAND_MAX) {
        value = (int32_t)CAR_MOTOR_COMMAND_MAX;
    }
    return (uint16_t)value;
}

static uint8_t Motor_IsValid(MotorId motor)
{
    return ((uint32_t)motor < (uint32_t)MOTOR_COUNT) ? 1U : 0U;
}

/*
 * 作用：设置 DIR 管脚。
 * 说明：当前默认 DIR 低为正转，高为反转；实车方向反了优先换接线表方向说明，
 *       或后续增加单电机方向反相配置，不在这里临时写死。
 */
static void Motor_ApplyDirection(MotorStepper *motor, MotorDir dir)
{
    if (dir == MOTOR_REVERSE) {
        DL_GPIO_setPins(motor->dirPort, motor->dirPin);
        motor->directionSign = -1;
    } else {
        DL_GPIO_clearPins(motor->dirPort, motor->dirPin);
        motor->directionSign = 1;
    }
}

/*
 * 作用：发出一个有界 STEP 脉冲。
 * 使用场景：Motor_Task 根据调度累加器补发脉冲。
 * 说明：这里只阻塞约 10us，且每轮有最大脉冲数限制，不会形成不可控死等。
 */
static void Motor_PulseStep(MotorStepper *motor)
{
    DL_GPIO_setPins(motor->stepPort, motor->stepPin);
    delay_cycles(CAR_STEPPER_PULSE_CYCLES);
    DL_GPIO_clearPins(motor->stepPort, motor->stepPin);
    motor->stepCount += (int32_t)motor->directionSign;
}

/*
 * 作用：刷新单个步进电机的脉冲调度。
 * 使用场景：Motor_Task 对四个电机轮询调用。
 * 说明：最大命令对应每次任务最多 CAR_STEPPER_MAX_STEPS_PER_TASK 个脉冲。
 */
static void Motor_TaskOne(MotorStepper *motor)
{
    const uint16_t quantum =
        (uint16_t)(CAR_MOTOR_COMMAND_MAX / CAR_STEPPER_MAX_STEPS_PER_TASK);
    uint8_t emitted = 0U;

    if ((motor->command == 0U) || (quantum == 0U)) {
        motor->accumulator = 0U;
        return;
    }

    motor->accumulator = (uint16_t)(motor->accumulator + motor->command);
    while ((motor->accumulator >= quantum) &&
        (emitted < CAR_STEPPER_MAX_STEPS_PER_TASK)) {
        motor->accumulator = (uint16_t)(motor->accumulator - quantum);
        Motor_PulseStep(motor);
        ++emitted;
    }
}

void Motor_Init(void)
{
    Motor_SetAllStop();
}

void Motor_Task(void)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)MOTOR_COUNT; ++i) {
        Motor_TaskOne(&g_motors[i]);
    }
}

void Motor_Set(MotorId motor, MotorDir dir, uint16_t command)
{
    MotorStepper *stepper;

    if (!Motor_IsValid(motor)) {
        return;
    }

    stepper = &g_motors[motor];
    if ((dir == MOTOR_COAST) || (dir == MOTOR_BRAKE) || (command == 0U)) {
        stepper->command = 0U;
        stepper->accumulator = 0U;
        DL_GPIO_clearPins(stepper->stepPort, stepper->stepPin);
        return;
    }

    Motor_ApplyDirection(stepper, dir);
    stepper->command = Motor_ClampCommand(command);
}

void Motor_SetChassisCommand(int16_t leftCommand, int16_t rightCommand)
{
    Motor_Set(MOTOR_CHASSIS_LEFT,
        (leftCommand >= 0) ? MOTOR_FORWARD : MOTOR_REVERSE,
        Motor_CommandFromSigned(leftCommand));
    Motor_Set(MOTOR_CHASSIS_RIGHT,
        (rightCommand >= 0) ? MOTOR_FORWARD : MOTOR_REVERSE,
        Motor_CommandFromSigned(rightCommand));
}

void Motor_SetAllStop(void)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)MOTOR_COUNT; ++i) {
        g_motors[i].command = 0U;
        g_motors[i].accumulator = 0U;
        g_motors[i].directionSign = 1;
        DL_GPIO_clearPins(g_motors[i].stepPort, g_motors[i].stepPin);
        DL_GPIO_clearPins(g_motors[i].dirPort, g_motors[i].dirPin);
    }
}

void Motor_Stop(void)
{
    Motor_SetAllStop();
}

int16_t Motor_GetCommand(MotorId motor)
{
    MotorStepper *stepper;

    if (!Motor_IsValid(motor)) {
        return 0;
    }
    stepper = &g_motors[motor];
    return (int16_t)((stepper->directionSign >= 0) ?
        (int16_t)stepper->command : -(int16_t)stepper->command);
}

int32_t Motor_GetStepCount(MotorId motor)
{
    if (!Motor_IsValid(motor)) {
        return 0;
    }
    return g_motors[motor].stepCount;
}
