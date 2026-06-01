#include "gimbal.h"

#include "board_config.h"
#include "motor.h"

typedef struct {
    GimbalPoint target;
    GimbalPoint current;
    int16_t errorX;
    int16_t errorY;
    int16_t commandX;
    int16_t commandY;
    uint16_t staleTicks;
    uint8_t enabled;
    uint8_t hasVision;
} GimbalControl;

static GimbalControl g_gimbal;

/*
 * 作用：把 int32_t 限制到 int16_t 可表达范围。
 * 使用场景：目标/当前位置相减，避免极端输入溢出。
 */
static int16_t Gimbal_ClampInt16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t)value;
}

static uint16_t Gimbal_Abs16(int16_t value)
{
    return (value < 0) ? (uint16_t)(-(int32_t)value) : (uint16_t)value;
}

/*
 * 作用：把视觉误差转换成某一轴的有符号速度命令。
 * 说明：误差在死区内不动；超过死区后按比例输出 STEP 命令。
 */
static int16_t Gimbal_ComputeAxisCommand(int16_t error, uint16_t deadband,
    uint16_t kp)
{
    uint16_t absError = Gimbal_Abs16(error);
    uint32_t command;

    if (absError <= deadband) {
        return 0;
    }

    command = ((uint32_t)(absError - deadband) * (uint32_t)kp) /
        (uint32_t)CAR_GIMBAL_GAIN_SCALE;
    if (command < CAR_GIMBAL_MIN_ACTIVE_COMMAND) {
        command = CAR_GIMBAL_MIN_ACTIVE_COMMAND;
    }
    if (command > CAR_GIMBAL_COMMAND_MAX) {
        command = CAR_GIMBAL_COMMAND_MAX;
    }
    if (command > CAR_MOTOR_COMMAND_MAX) {
        command = CAR_MOTOR_COMMAND_MAX;
    }

    return (error < 0) ? (int16_t)(-(int32_t)command) : (int16_t)command;
}

static int16_t Gimbal_ApplyReverse(int16_t command, uint8_t reverse)
{
    return reverse ? (int16_t)(-command) : command;
}

/*
 * 作用：把有符号命令输出给单个云台步进轴。
 * 说明：0 命令只停当前轴，不调用 Motor_Stop，避免误停底盘。
 */
static void Gimbal_SetAxis(MotorId motor, int16_t command)
{
    if (command > 0) {
        Motor_Set(motor, MOTOR_FORWARD, (uint16_t)command);
    } else if (command < 0) {
        Motor_Set(motor, MOTOR_REVERSE, Gimbal_Abs16(command));
    } else {
        Motor_Set(motor, MOTOR_COAST, 0U);
    }
}

static void Gimbal_UpdateError(void)
{
    g_gimbal.errorX = Gimbal_ClampInt16(
        (int32_t)g_gimbal.target.x - (int32_t)g_gimbal.current.x);
    g_gimbal.errorY = Gimbal_ClampInt16(
        (int32_t)g_gimbal.target.y - (int32_t)g_gimbal.current.y);
}

static void Gimbal_ApplyControl(void)
{
    int16_t commandX;
    int16_t commandY;

    commandX = Gimbal_ComputeAxisCommand(g_gimbal.errorX,
        CAR_GIMBAL_DEADBAND_X, CAR_GIMBAL_X_KP);
    commandY = Gimbal_ComputeAxisCommand(g_gimbal.errorY,
        CAR_GIMBAL_DEADBAND_Y, CAR_GIMBAL_Y_KP);

    commandX = Gimbal_ApplyReverse(commandX, CAR_GIMBAL_X_REVERSE);
    commandY = Gimbal_ApplyReverse(commandY, CAR_GIMBAL_Y_REVERSE);

    g_gimbal.commandX = commandX;
    g_gimbal.commandY = commandY;

    /* X 视觉误差控制左右轴，Y 视觉误差控制上下轴。 */
    Gimbal_SetAxis(MOTOR_GIMBAL_1, commandX);
    Gimbal_SetAxis(MOTOR_GIMBAL_2, commandY);
}

void Gimbal_Init(void)
{
    g_gimbal.target.x = 0;
    g_gimbal.target.y = 0;
    g_gimbal.current.x = 0;
    g_gimbal.current.y = 0;
    g_gimbal.errorX = 0;
    g_gimbal.errorY = 0;
    g_gimbal.commandX = 0;
    g_gimbal.commandY = 0;
    g_gimbal.staleTicks = 0U;
    g_gimbal.enabled = 0U;
    g_gimbal.hasVision = 0U;
    Gimbal_Stop();
}

void Gimbal_SetEnabled(uint8_t enabled)
{
    g_gimbal.enabled = enabled ? 1U : 0U;
    if (!g_gimbal.enabled) {
        Gimbal_Stop();
    }
}

uint8_t Gimbal_IsEnabled(void)
{
    return g_gimbal.enabled;
}

void Gimbal_SetTarget(int16_t x, int16_t y)
{
    g_gimbal.target.x = x;
    g_gimbal.target.y = y;
    Gimbal_UpdateError();
}

void Gimbal_SetCurrent(int16_t x, int16_t y)
{
    g_gimbal.current.x = x;
    g_gimbal.current.y = y;
    g_gimbal.hasVision = 1U;
    g_gimbal.staleTicks = 0U;
    Gimbal_UpdateError();
}

void Gimbal_UpdateFromVision(int16_t targetX, int16_t targetY,
    int16_t currentX, int16_t currentY)
{
    g_gimbal.target.x = targetX;
    g_gimbal.target.y = targetY;
    g_gimbal.current.x = currentX;
    g_gimbal.current.y = currentY;
    g_gimbal.hasVision = 1U;
    g_gimbal.staleTicks = 0U;
    Gimbal_UpdateError();
}

void Gimbal_Task(void)
{
    if (!g_gimbal.enabled) {
        return;
    }

    if (!g_gimbal.hasVision) {
        Gimbal_Stop();
        return;
    }

    if (g_gimbal.staleTicks >= CAR_GIMBAL_VISION_TIMEOUT_TICKS) {
        Gimbal_Stop();
        return;
    }
    ++g_gimbal.staleTicks;

    Gimbal_ApplyControl();
}

void Gimbal_Stop(void)
{
    g_gimbal.commandX = 0;
    g_gimbal.commandY = 0;
    Gimbal_SetAxis(MOTOR_GIMBAL_1, 0);
    Gimbal_SetAxis(MOTOR_GIMBAL_2, 0);
}

int16_t Gimbal_GetErrorX(void)
{
    return g_gimbal.errorX;
}

int16_t Gimbal_GetErrorY(void)
{
    return g_gimbal.errorY;
}

int16_t Gimbal_GetCommandX(void)
{
    return g_gimbal.commandX;
}

int16_t Gimbal_GetCommandY(void)
{
    return g_gimbal.commandY;
}
