#include "motor_no_yaw.h"

#include "board_config.h"
#include "gray.h"
#include "log_uart.h"
#include "motion.h"
#include "motor.h"
#include "motor_enable.h"

#define MOTOR_NO_YAW_CENTER_MASK (0x08U)

typedef struct {
    int32_t leftBaseStep;
    int32_t rightBaseStep;
    uint32_t travelSteps;
    int16_t lineError;
    int16_t lastLeftSpeedSps;
    int16_t lastRightSpeedSps;
    uint16_t turnTicks;
    uint16_t lineLostTicks;
    uint8_t phase;
    uint8_t digitalMask;
    uint8_t hasLastLineCommand;
    uint8_t running;
    MotorNoYawState state;
} MotorNoYawControl;

static MotorNoYawControl g_motorNoYaw;

/* 作用：把 ms 转成 App_Task 调度 tick，向上取整。 */
static uint16_t MotorNoYaw_MsToTicks(uint16_t timeMs)
{
    return (uint16_t)((timeMs + CAR_APP_LOOP_DELAY_MS - 1U) /
        CAR_APP_LOOP_DELAY_MS);
}

/* 作用：计算 int32_t 绝对值，用于把正反向 STEP 都转成路程。 */
static uint32_t MotorNoYaw_Abs32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

/* 作用：限制循迹差速修正，避免基础速度被修正量完全吃掉。 */
static int16_t MotorNoYaw_ClampCorrection(int32_t correction, uint16_t limit)
{
    if (correction > (int32_t)limit) {
        return (int16_t)limit;
    }
    if (correction < -(int32_t)limit) {
        return (int16_t)(-(int32_t)limit);
    }
    return (int16_t)correction;
}

/* 作用：把当前左右轮 STEP 计数作为新一段灰度循迹直行的起点。 */
static void MotorNoYaw_ResetSegmentSteps(void)
{
    g_motorNoYaw.leftBaseStep = Motor_GetStepCount(MOTOR_CHASSIS_LEFT);
    g_motorNoYaw.rightBaseStep = Motor_GetStepCount(MOTOR_CHASSIS_RIGHT);
    g_motorNoYaw.travelSteps = 0U;
}

/* 作用：读取当前段左右轮平均 STEP，正反转都按路程累计。 */
static uint32_t MotorNoYaw_UpdateTravelSteps(void)
{
    int32_t leftDelta =
        Motor_GetStepCount(MOTOR_CHASSIS_LEFT) - g_motorNoYaw.leftBaseStep;
    int32_t rightDelta =
        Motor_GetStepCount(MOTOR_CHASSIS_RIGHT) - g_motorNoYaw.rightBaseStep;

    g_motorNoYaw.travelSteps =
        (MotorNoYaw_Abs32(leftDelta) + MotorNoYaw_Abs32(rightDelta)) / 2U;
    return g_motorNoYaw.travelSteps;
}

/* 作用：更新灰度数据并缓存 bitmask/lineError。 */
static uint8_t MotorNoYaw_UpdateGray(void)
{
    int16_t lineError = 0;

    if (Gray_Update() == 0U) {
        return 0U;
    }

    g_motorNoYaw.digitalMask = Gray_GetDigitalMask();
    if (Gray_GetWeightedLineError(&lineError) != 0U) {
        g_motorNoYaw.lineError = lineError;
    } else if ((g_motorNoYaw.digitalMask & MOTOR_NO_YAW_CENTER_MASK) != 0U) {
        g_motorNoYaw.lineError = 0;
    }

    return 1U;
}

/* 作用：按灰度误差输出一轮差速循迹命令。 */
static void MotorNoYaw_ApplyLineControl(void)
{
    int16_t baseSpeed = (int16_t)CAR_MOTOR_NO_YAW_BASE_SPEED_SPS;
    int16_t correction = MotorNoYaw_ClampCorrection(
        ((int32_t)g_motorNoYaw.lineError * (int32_t)CAR_TRACK_TURN_GAIN) /
            (int32_t)GRAY_LINE_ERROR_SCALE,
        CAR_MOTOR_NO_YAW_BASE_SPEED_SPS);
    int16_t leftSpeed = (int16_t)(baseSpeed + correction);
    int16_t rightSpeed = (int16_t)(baseSpeed - correction);

    g_motorNoYaw.lastLeftSpeedSps = leftSpeed;
    g_motorNoYaw.lastRightSpeedSps = rightSpeed;
    g_motorNoYaw.hasLastLineCommand = 1U;
    Motion_SetChassisCommand(leftSpeed, rightSpeed);
}

/* 作用：异常停车并保留页面，等待 K2 长按返回菜单。 */
static void MotorNoYaw_StopWithReason(const char *reason)
{
    Motion_SetChassisCommand(0, 0);
    MotorEnable_SetChassis(CAR_STEPPER_ENABLE_DEFAULT_ON);
    g_motorNoYaw.running = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_STOP;

    LOG_RAW("[NO YAW] stop ");
    LOG_RAW(reason);
    LOG_RAW(" step=");
    LogUart_SendUnsigned(g_motorNoYaw.travelSteps);
    LOG_RAW(" mask=");
    LogUart_SendHex32(g_motorNoYaw.digitalMask);
    LOG_RAW(" err=");
    LogUart_SendSigned(g_motorNoYaw.lineError);
    LOG_LINE("");
}

/* 作用：进入开环原地右转；左轮前进，右轮后退。 */
static void MotorNoYaw_StartRightTurn(void)
{
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.hasLastLineCommand = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_TURN_RIGHT;
    LOG_RAW("[NO YAW] open turn step=");
    LogUart_SendUnsigned(g_motorNoYaw.travelSteps);
    LOG_LINE("");
}

/* 作用：开环右转结束后直接恢复灰度循迹，并重新计下一段 STEP。 */
static void MotorNoYaw_FinishTurn(void)
{
    g_motorNoYaw.phase = (uint8_t)((g_motorNoYaw.phase + 1U) & 0x03U);
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.hasLastLineCommand = 0U;
    MotorNoYaw_ResetSegmentSteps();
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_LINE;
    Motion_Forward(CAR_MOTOR_NO_YAW_BASE_SPEED_SPS);
    LOG_RAW("[NO YAW] open turn done phase=");
    LogUart_SendUnsigned(g_motorNoYaw.phase);
    LOG_LINE("");
}

/* 作用：灰度循迹直行，到固定 STEP 后开环右转；不再识别侧边全黑。 */
static void MotorNoYaw_TaskLine(void)
{
    uint16_t lineLostTimeoutTicks =
        MotorNoYaw_MsToTicks(CAR_MOTOR_NO_YAW_LINE_LOST_TIMEOUT_MS);

    if (MotorNoYaw_UpdateGray() == 0U) {
        MotorNoYaw_StopWithReason("gray-fault");
        return;
    }

    if (g_motorNoYaw.digitalMask == 0U) {
        if (g_motorNoYaw.hasLastLineCommand != 0U) {
            Motion_SetChassisCommand(g_motorNoYaw.lastLeftSpeedSps,
                g_motorNoYaw.lastRightSpeedSps);
        } else {
            Motion_SetChassisCommand(0, 0);
        }
        if (g_motorNoYaw.lineLostTicks >= lineLostTimeoutTicks) {
            MotorNoYaw_StopWithReason("line-lost");
            return;
        }
        ++g_motorNoYaw.lineLostTicks;
    } else {
        g_motorNoYaw.lineLostTicks = 0U;
        MotorNoYaw_ApplyLineControl();
    }

    if (MotorNoYaw_UpdateTravelSteps() >=
        (uint32_t)CAR_MOTOR_NO_YAW_OPEN_LOOP_STEPS) {
        MotorNoYaw_StartRightTurn();
    }
}

/* 作用：开环原地右转固定时间，到时直接恢复灰度循迹。 */
static void MotorNoYaw_TaskTurning(void)
{
    uint16_t holdTicks =
        MotorNoYaw_MsToTicks(CAR_MOTOR_NO_YAW_TURN_HOLD_MS);

    Motion_SetChassisCommand((int16_t)CAR_MOTOR_NO_YAW_TURN_SPEED_SPS,
        (int16_t)(-(int32_t)CAR_MOTOR_NO_YAW_TURN_SPEED_SPS));

    if (g_motorNoYaw.turnTicks < 0xFFFFU) {
        ++g_motorNoYaw.turnTicks;
    }
    if (g_motorNoYaw.turnTicks >= holdTicks) {
        MotorNoYaw_FinishTurn();
    }
}

void MotorNoYaw_Init(void)
{
    g_motorNoYaw.leftBaseStep = 0;
    g_motorNoYaw.rightBaseStep = 0;
    g_motorNoYaw.travelSteps = 0U;
    g_motorNoYaw.lineError = 0;
    g_motorNoYaw.lastLeftSpeedSps = 0;
    g_motorNoYaw.lastRightSpeedSps = 0;
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.phase = 0U;
    g_motorNoYaw.digitalMask = 0U;
    g_motorNoYaw.hasLastLineCommand = 0U;
    g_motorNoYaw.running = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_IDLE;
}

void MotorNoYaw_Start(void)
{
    Motor_ResetStepCount(MOTOR_CHASSIS_LEFT);
    Motor_ResetStepCount(MOTOR_CHASSIS_RIGHT);
    MotorNoYaw_ResetSegmentSteps();
    g_motorNoYaw.lineError = 0;
    g_motorNoYaw.lastLeftSpeedSps = 0;
    g_motorNoYaw.lastRightSpeedSps = 0;
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.phase = 0U;
    g_motorNoYaw.digitalMask = 0U;
    g_motorNoYaw.hasLastLineCommand = 0U;
    g_motorNoYaw.running = 1U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_LINE;
    MotorEnable_SetChassis(1U);
    LOG_LINE("[NO YAW] gray line + open turn start");
}

void MotorNoYaw_Stop(void)
{
    if (g_motorNoYaw.state != MOTOR_NO_YAW_STATE_IDLE) {
        Motion_SetChassisCommand(0, 0);
        MotorEnable_SetChassis(CAR_STEPPER_ENABLE_DEFAULT_ON);
    }
    g_motorNoYaw.running = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_IDLE;
}

void MotorNoYaw_Task(void)
{
    if (g_motorNoYaw.running == 0U) {
        return;
    }

    switch (g_motorNoYaw.state) {
    case MOTOR_NO_YAW_STATE_LINE:
        MotorNoYaw_TaskLine();
        break;
    case MOTOR_NO_YAW_STATE_TURN_RIGHT:
        MotorNoYaw_TaskTurning();
        break;
    case MOTOR_NO_YAW_STATE_TURN_LEFT:
    case MOTOR_NO_YAW_STATE_STOP:
    case MOTOR_NO_YAW_STATE_IDLE:
    default:
        break;
    }
}

uint8_t MotorNoYaw_IsRunning(void)
{
    return g_motorNoYaw.running;
}

MotorNoYawState MotorNoYaw_GetState(void)
{
    return g_motorNoYaw.state;
}

const char *MotorNoYaw_GetStateName(void)
{
    switch (g_motorNoYaw.state) {
    case MOTOR_NO_YAW_STATE_LINE:
        return "Line";
    case MOTOR_NO_YAW_STATE_TURN_RIGHT:
        return "RSpin";
    case MOTOR_NO_YAW_STATE_TURN_LEFT:
        return "LSpin";
    case MOTOR_NO_YAW_STATE_STOP:
        return "Stop";
    case MOTOR_NO_YAW_STATE_IDLE:
    default:
        return "Idle";
    }
}

uint8_t MotorNoYaw_GetPhase(void)
{
    return g_motorNoYaw.phase;
}

uint32_t MotorNoYaw_GetTravelSteps(void)
{
    return g_motorNoYaw.travelSteps;
}

uint8_t MotorNoYaw_GetDigitalMask(void)
{
    return g_motorNoYaw.digitalMask;
}

int16_t MotorNoYaw_GetLineError(void)
{
    return g_motorNoYaw.lineError;
}
