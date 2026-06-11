#include "gimbal.h"

#include "board_config.h"
#include "motor.h"
#include "staticconfig.h"

#if (CAR_GIMBAL_PITCH_STEPS_PER_90 == 0U)
#error "CAR_GIMBAL_PITCH_STEPS_PER_90 must be greater than 0"
#endif

typedef struct {
    /* target/current 使用同一个视觉坐标系，当前视觉输入单位为 0.1 像素。 */
    GimbalPoint target;
    GimbalPoint current;
    /* error = target - current，正负号决定云台转动方向。 */
    int16_t errorX;
    int16_t errorY;
    int16_t errorDeltaX;
    int16_t errorDeltaY;
    int16_t lastErrorX;
    int16_t lastErrorY;
    /* command 是输出给 motor.c 的有符号 SPS，绝对值越大 STEP 越快。 */
    int16_t commandX;
    int16_t commandY;
    /* staleTicks 用来做视觉掉线保护，长时间没新数据就停云台。 */
    uint16_t staleTicks;
    /* pitchBaseStep 是进入视觉闭环时的上下轴 STEP 计数，作为相对 0 度。 */
    int32_t pitchBaseStep;
    uint8_t enabled;
    uint8_t hasVision;
    uint8_t hasLastError;
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

/* 作用：计算 int16_t 绝对值，用于命令限幅和方向拆分。 */
static uint16_t Gimbal_Abs16(int16_t value)
{
    return (value < 0) ? (uint16_t)(-(int32_t)value) : (uint16_t)value;
}

/*
 * 作用：把视觉误差转换成某一轴的有符号 PD SPS。
 * 说明：P 负责响应，D 根据相邻视觉帧的误差变化做阻尼，不累加误差。
 */
static int16_t Gimbal_ComputeAxisCommand(int16_t error, int16_t errorDelta,
    uint16_t deadband, uint16_t kp, uint16_t kd, uint16_t gainScale,
    uint16_t minSpeedSps, uint16_t maxSpeedSps)
{
    uint16_t absError = Gimbal_Abs16(error);
    int32_t command;
    uint32_t commandAbs;

    if (absError <= deadband) {
        return 0;
    }
    if (gainScale == 0U) {
        gainScale = 1U;
    }

    command = (((int32_t)error * (int32_t)kp) +
        ((int32_t)errorDelta * (int32_t)kd)) /
        (int32_t)gainScale;

    if (command == 0) {
        commandAbs = minSpeedSps;
    } else {
        if (((error > 0) && (command < 0)) ||
            ((error < 0) && (command > 0))) {
            return 0;
        }

        commandAbs = (uint32_t)Gimbal_Abs16(Gimbal_ClampInt16(command));
        if (commandAbs < (uint32_t)minSpeedSps) {
            commandAbs = minSpeedSps;
        }
    }

    if (commandAbs > (uint32_t)maxSpeedSps) {
        commandAbs = maxSpeedSps;
    }
    if (commandAbs > (uint32_t)CAR_STEPPER_SPEED_MAX_SPS) {
        commandAbs = CAR_STEPPER_SPEED_MAX_SPS;
    }

    return (error < 0) ? (int16_t)(-(int32_t)commandAbs) :
        (int16_t)commandAbs;
}

/* 作用：根据配置宏对某个云台轴的方向取反。 */
static int16_t Gimbal_ApplyReverse(int16_t command, uint8_t reverse)
{
    return reverse ? (int16_t)(-command) : command;
}

/* 作用：计算 pitch 限幅对应的 STEP 数。 */
static int32_t Gimbal_GetPitchLimitSteps(void)
{
    return ((int32_t)CAR_GIMBAL_PITCH_STEPS_PER_90 *
        (int32_t)CAR_GIMBAL_PITCH_LIMIT_DEG) / 90;
}

/*
 * 作用：按 pitch 相对角限幅裁剪上下轴命令。
 * 说明：没有编码器/回零开关时，只能用进入闭环时的 STEP 计数作为相对零点。
 */
static int16_t Gimbal_LimitPitchCommand(int16_t command)
{
    int32_t pitchDelta =
        Motor_GetStepCount(MOTOR_GIMBAL_2) - g_gimbal.pitchBaseStep;
    int32_t limitSteps = Gimbal_GetPitchLimitSteps();

    if ((command > 0) && (pitchDelta >= limitSteps)) {
        return 0;
    }
    if ((command < 0) && (pitchDelta <= -limitSteps)) {
        return 0;
    }
    return command;
}

/*
 * 作用：把有符号 SPS 输出给单个云台步进轴。
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

/*
 * 作用：写入最新视觉误差，并记录相邻视觉帧的误差变化量。
 * 使用场景：target/current 或激光差值变化后调用。
 */
static void Gimbal_SetError(int16_t errorX, int16_t errorY)
{
    if (g_gimbal.hasLastError != 0U) {
        g_gimbal.errorDeltaX = Gimbal_ClampInt16(
            (int32_t)errorX - (int32_t)g_gimbal.lastErrorX);
        g_gimbal.errorDeltaY = Gimbal_ClampInt16(
            (int32_t)errorY - (int32_t)g_gimbal.lastErrorY);
    } else {
        g_gimbal.errorDeltaX = 0;
        g_gimbal.errorDeltaY = 0;
        g_gimbal.hasLastError = 1U;
    }

    g_gimbal.errorX = errorX;
    g_gimbal.errorY = errorY;
    g_gimbal.lastErrorX = errorX;
    g_gimbal.lastErrorY = errorY;
}

/* 作用：根据 target/current 刷新误差。 */
static void Gimbal_UpdateError(void)
{
    Gimbal_SetError(
        Gimbal_ClampInt16(
            (int32_t)g_gimbal.target.x - (int32_t)g_gimbal.current.x),
        Gimbal_ClampInt16(
            (int32_t)g_gimbal.target.y - (int32_t)g_gimbal.current.y));
}

/*
 * 作用：把当前视觉误差换算成两个云台轴的 SPS。
 * 使用场景：Gimbal_Task 检查视觉数据有效后调用。
 * 说明：这里只写 motor 速度，不读取传感器，也不做串口解析。
 */
static void Gimbal_ApplyControl(void)
{
    const StaticConfigGimbalTask *config = StaticConfig_GetActiveGimbal();
    int16_t commandX;
    int16_t commandY;

    commandX = Gimbal_ComputeAxisCommand(g_gimbal.errorX,
        g_gimbal.errorDeltaX, config->deadbandX, config->kpX, config->kdX,
        config->gainScale, config->minSpeedX, config->maxSpeedX);
    commandY = Gimbal_ComputeAxisCommand(g_gimbal.errorY,
        g_gimbal.errorDeltaY, config->deadbandY, config->kpY, config->kdY,
        config->gainScale, config->minSpeedY, config->maxSpeedY);

    commandX = Gimbal_ApplyReverse(commandX, CAR_GIMBAL_X_REVERSE);
    commandY = Gimbal_ApplyReverse(commandY, CAR_GIMBAL_Y_REVERSE);
    commandY = Gimbal_LimitPitchCommand(commandY);

    g_gimbal.commandX = commandX;
    g_gimbal.commandY = commandY;

    /* X 视觉误差控制左右轴，Y 视觉误差控制上下轴。 */
    Gimbal_SetAxis(MOTOR_GIMBAL_1, commandX);
    Gimbal_SetAxis(MOTOR_GIMBAL_2, commandY);
}

/*
 * 作用：初始化二维云台控制器。
 * 使用场景：App_Init 调用一次。
 * 说明：初始化后闭环关闭，两个云台轴保持停止。
 */
void Gimbal_Init(void)
{
    g_gimbal.target.x = 0;
    g_gimbal.target.y = 0;
    g_gimbal.current.x = 0;
    g_gimbal.current.y = 0;
    g_gimbal.errorX = 0;
    g_gimbal.errorY = 0;
    g_gimbal.errorDeltaX = 0;
    g_gimbal.errorDeltaY = 0;
    g_gimbal.lastErrorX = 0;
    g_gimbal.lastErrorY = 0;
    g_gimbal.commandX = 0;
    g_gimbal.commandY = 0;
    g_gimbal.staleTicks = 0U;
    g_gimbal.pitchBaseStep = Motor_GetStepCount(MOTOR_GIMBAL_2);
    g_gimbal.enabled = 0U;
    g_gimbal.hasVision = 0U;
    g_gimbal.hasLastError = 0U;
    Gimbal_Stop();
}

/*
 * 作用：启用或关闭云台闭环。
 * 使用场景：Vision Test、仿真视觉测试进入/退出时。
 * 说明：关闭时会立即停止云台两个轴，但不影响底盘电机。
 */
void Gimbal_SetEnabled(uint8_t enabled)
{
    uint8_t nextEnabled = enabled ? 1U : 0U;

    if ((g_gimbal.enabled == 0U) && (nextEnabled != 0U)) {
        g_gimbal.pitchBaseStep = Motor_GetStepCount(MOTOR_GIMBAL_2);
        g_gimbal.staleTicks = 0U;
        g_gimbal.hasVision = 0U;
        g_gimbal.errorDeltaX = 0;
        g_gimbal.errorDeltaY = 0;
        g_gimbal.hasLastError = 0U;
    }

    g_gimbal.enabled = nextEnabled;
    if (!g_gimbal.enabled) {
        Gimbal_Stop();
    }
}

/* 作用：返回云台闭环是否启用。 */
uint8_t Gimbal_IsEnabled(void)
{
    return g_gimbal.enabled;
}

/* 作用：单独更新目标点，坐标单位跟视觉输入一致，当前为 0.1 像素。 */
void Gimbal_SetTarget(int16_t x, int16_t y)
{
    g_gimbal.target.x = x;
    g_gimbal.target.y = y;
    Gimbal_UpdateError();
}

/* 作用：单独更新当前识别点，并标记已有视觉数据，当前单位为 0.1 像素。 */
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
    /*
     * 这里只更新控制输入和误差，不直接等待或阻塞。
     * 真正的 STEP 命令在 Gimbal_Task() 里周期输出。
     */
    g_gimbal.target.x = targetX;
    g_gimbal.target.y = targetY;
    g_gimbal.current.x = currentX;
    g_gimbal.current.y = currentY;
    g_gimbal.hasVision = 1U;
    g_gimbal.staleTicks = 0U;
    Gimbal_UpdateError();
}

void Gimbal_UpdateFromLaserError(int16_t laserMinusTargetX,
    int16_t laserMinusTargetY)
{
    /*
     * 视觉脚本发的是 laser - target，也就是激光点相对矩形中心的差值。
     * 当前调用方已把输入转成 0.1 像素单位。
     * 云台控制内部使用 target - current，因此这里统一取反。
     */
    Gimbal_UpdateFromCameraError(
        Gimbal_ClampInt16(-(int32_t)laserMinusTargetX),
        Gimbal_ClampInt16(-(int32_t)laserMinusTargetY));
}

void Gimbal_UpdateFromCameraError(int16_t targetMinusCurrentX,
    int16_t targetMinusCurrentY)
{
    const StaticConfigGimbalTask *config = StaticConfig_GetActiveGimbal();
    int16_t adjustedErrorX;
    int16_t adjustedErrorY;

    /*
     * 新视觉脚本发的是 160-x,120-y，已经是 target - current。
     * 安装偏差补偿只改控制目标，不改视觉端识别坐标。
     */
    adjustedErrorX = Gimbal_ClampInt16(
        (int32_t)targetMinusCurrentX +
        (int32_t)config->offsetX);
    adjustedErrorY = Gimbal_ClampInt16(
        (int32_t)targetMinusCurrentY +
        (int32_t)config->offsetY);

    g_gimbal.target.x = 0;
    g_gimbal.target.y = 0;
    g_gimbal.current.x = Gimbal_ClampInt16(
        -(int32_t)adjustedErrorX);
    g_gimbal.current.y = Gimbal_ClampInt16(
        -(int32_t)adjustedErrorY);
    Gimbal_SetError(adjustedErrorX, adjustedErrorY);
    g_gimbal.hasVision = 1U;
    g_gimbal.staleTicks = 0U;
}

/*
 * 作用：云台闭环周期任务。
 * 使用场景：App_Task 每轮调用。
 * 说明：没有视觉数据或视觉超时时会停止云台，防止丢帧后继续输出旧命令。
 */
void Gimbal_Task(void)
{
    if (!g_gimbal.enabled) {
        return;
    }

    /* 启用了闭环但还没有视觉数据时，保持云台停止。 */
    if (!g_gimbal.hasVision) {
        Gimbal_Stop();
        return;
    }

    /* 视觉数据超时后停止输出，避免目标丢失时云台继续乱转。 */
    if (g_gimbal.staleTicks >= CAR_GIMBAL_VISION_TIMEOUT_TICKS) {
        Gimbal_Stop();
        return;
    }
    ++g_gimbal.staleTicks;

    Gimbal_ApplyControl();
}

/* 作用：停止云台两个轴，不改变底盘速度。 */
void Gimbal_Stop(void)
{
    g_gimbal.commandX = 0;
    g_gimbal.commandY = 0;
    Gimbal_SetAxis(MOTOR_GIMBAL_1, 0);
    Gimbal_SetAxis(MOTOR_GIMBAL_2, 0);
}

/* 作用：返回 X 方向视觉误差，符号为 target - current，单位为 0.1 像素。 */
int16_t Gimbal_GetErrorX(void)
{
    return g_gimbal.errorX;
}

/* 作用：返回 Y 方向视觉误差，符号为 target - current，单位为 0.1 像素。 */
int16_t Gimbal_GetErrorY(void)
{
    return g_gimbal.errorY;
}

/* 作用：返回最近一次输出到左右轴的有符号 SPS。 */
int16_t Gimbal_GetCommandX(void)
{
    return g_gimbal.commandX;
}

/* 作用：返回最近一次输出到上下轴的有符号 SPS。 */
int16_t Gimbal_GetCommandY(void)
{
    return g_gimbal.commandY;
}
