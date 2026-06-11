#include "motor_track.h"

#include "board_config.h"
#include "gray.h"
#include "jy61p.h"
#include "log_uart.h"
#include "motion.h"
#include "motor.h"
#include "motor_enable.h"

#define MOTOR_TRACK_CENTER_MASK (0x08U)

typedef struct {
    int32_t leftBaseStep;
    int32_t rightBaseStep;
    uint32_t travelSteps;
    uint16_t baseSpeedSps;
    uint16_t turnTicks;
    uint16_t exitSlowTicks;
    int16_t lineError;
    int16_t yawStartDeg;
    int16_t yawDeltaDeg;
    uint8_t phase;
    uint8_t digitalMask;
    uint8_t running;
    MotorTrackState state;
} MotorTrackControl;

static MotorTrackControl g_motorTrack;

/* 作用：把 ms 转成 App_Task 调度 tick，向上取整。 */
static uint16_t MotorTrack_MsToTicks(uint16_t timeMs)
{
    return (uint16_t)((timeMs + CAR_APP_LOOP_DELAY_MS - 1U) /
        CAR_APP_LOOP_DELAY_MS);
}

/* 作用：计算 int32_t 绝对值，用于把正反向 STEP 都转成路程。 */
static uint32_t MotorTrack_Abs32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

/* 作用：计算 int16_t 绝对值，用于 yaw 变化幅度。 */
static int16_t MotorTrack_Abs16(int16_t value)
{
    return (value < 0) ? (int16_t)(-(int32_t)value) : value;
}

/* 作用：限制循迹差速修正，避免基础速度被修正量完全吃掉。 */
static int16_t MotorTrack_ClampCorrection(int32_t correction,
    uint16_t limit)
{
    if (correction > (int32_t)limit) {
        return (int16_t)limit;
    }
    if (correction < -(int32_t)limit) {
        return (int16_t)(-(int32_t)limit);
    }
    return (int16_t)correction;
}

/* 作用：把 yaw 差值归一到 -180~180 度，处理 359/0 度环绕。 */
static int16_t MotorTrack_NormalizeYawDelta(int16_t currentYawDeg,
    int16_t startYawDeg)
{
    int16_t delta = (int16_t)(currentYawDeg - startYawDeg);

    while (delta > 180) {
        delta = (int16_t)(delta - 360);
    }
    while (delta < -180) {
        delta = (int16_t)(delta + 360);
    }
    return delta;
}

/* 作用：读取左右轮相对当前段起点的平均 STEP 数。 */
static uint32_t MotorTrack_UpdateTravelSteps(void)
{
    int32_t leftDelta =
        Motor_GetStepCount(MOTOR_CHASSIS_LEFT) - g_motorTrack.leftBaseStep;
    int32_t rightDelta =
        Motor_GetStepCount(MOTOR_CHASSIS_RIGHT) - g_motorTrack.rightBaseStep;

    g_motorTrack.travelSteps =
        (MotorTrack_Abs32(leftDelta) + MotorTrack_Abs32(rightDelta)) / 2U;
    return g_motorTrack.travelSteps;
}

/* 作用：把当前左右轮 STEP 计数作为新一段的起点。 */
static void MotorTrack_ResetSegmentSteps(void)
{
    g_motorTrack.leftBaseStep = Motor_GetStepCount(MOTOR_CHASSIS_LEFT);
    g_motorTrack.rightBaseStep = Motor_GetStepCount(MOTOR_CHASSIS_RIGHT);
    g_motorTrack.travelSteps = 0U;
}

/* 作用：判断右转灰度信号；路线只认右转，左转信号直接忽略。 */
static uint8_t MotorTrack_IsRightTurnDetected(uint8_t digitalMask)
{
    return ((digitalMask & CAR_MOTOR_TRACK_RIGHT_TURN_MASK) ==
        CAR_MOTOR_TRACK_RIGHT_TURN_MASK) ? 1U : 0U;
}

/* 作用：更新一次灰度数据并缓存 lineError/digitalMask。 */
static uint8_t MotorTrack_UpdateGray(void)
{
    int16_t lineError = 0;
    uint8_t hasLineError;

    if (Gray_Update() == 0U) {
        return 0U;
    }

    g_motorTrack.digitalMask = Gray_GetDigitalMask();
    hasLineError = Gray_GetWeightedLineError(&lineError);
    if (hasLineError != 0U) {
        g_motorTrack.lineError = lineError;
    } else if ((g_motorTrack.digitalMask & MOTOR_TRACK_CENTER_MASK) != 0U) {
        g_motorTrack.lineError = 0;
    }
    return 1U;
}

/* 作用：按红外误差输出一轮差速循迹命令。 */
static void MotorTrack_ApplyLineControl(uint16_t baseSpeedSps)
{
    int16_t baseSpeed = (int16_t)baseSpeedSps;
    int16_t correction = MotorTrack_ClampCorrection(
        ((int32_t)g_motorTrack.lineError * (int32_t)CAR_TRACK_TURN_GAIN) /
            (int32_t)GRAY_LINE_ERROR_SCALE,
        baseSpeedSps);

    Motion_SetChassisCommand((int16_t)(baseSpeed + correction),
        (int16_t)(baseSpeed - correction));
}

/* 作用：真循迹异常停车并保留页面，等待 K2 长按返回菜单。 */
static void MotorTrack_StopWithReason(const char *reason)
{
    Motion_SetChassisCommand(0, 0);
    MotorEnable_SetChassis(CAR_STEPPER_ENABLE_DEFAULT_ON);
    g_motorTrack.running = 0U;
    g_motorTrack.state = MOTOR_TRACK_STATE_STOP;

    LOG_RAW("[MOTOR TRACK] stop ");
    LOG_RAW(reason);
    LOG_RAW(" phase=");
    LogUart_SendUnsigned(g_motorTrack.phase);
    LOG_RAW(" step=");
    LogUart_SendUnsigned(g_motorTrack.travelSteps);
    LOG_RAW(" yaw=");
    LogUart_SendSigned(g_motorTrack.yawDeltaDeg);
    LOG_LINE("");
}

/*
 * 作用：预留云台相位切换接口。
 * 说明：当前只打印 phase，后续在这里映射四个 0/1/2/3 状态到云台参数。
 */
static void MotorTrack_OnPhaseChanged(uint8_t phase)
{
    LOG_RAW("[MOTOR TRACK] phase ");
    LogUart_SendUnsigned(phase);
    LOG_LINE("");
}

/*
 * 作用：预留云台姿态补偿接口。
 * 说明：当前不改云台，后续可把右转 yawDelta 转成云台补偿量。
 */
static void MotorTrack_OnTurnYawDelta(int16_t yawDeltaDeg)
{
    (void)yawDeltaDeg;
}

/* 作用：从低速循迹进入原地强右转。 */
static void MotorTrack_StartRightTurn(void)
{
    if (JY61P_HasYaw() == 0U) {
        MotorTrack_StopWithReason("yaw-missing");
        return;
    }

    g_motorTrack.yawStartDeg = JY61P_GetYawDeg();
    g_motorTrack.yawDeltaDeg = 0;
    g_motorTrack.turnTicks = 0U;
    g_motorTrack.state = MOTOR_TRACK_STATE_TURNING_SPIN;
    LOG_RAW("[MOTOR TRACK] right turn yawStart=");
    LogUart_SendSigned(g_motorTrack.yawStartDeg);
    LOG_LINE("");
}

/* 作用：执行快/慢直线红外循迹。 */
static void MotorTrack_RunLineState(uint16_t baseSpeedSps)
{
    if (MotorTrack_UpdateGray() == 0U) {
        MotorTrack_StopWithReason("gray-fault");
        return;
    }

    MotorTrack_ApplyLineControl(baseSpeedSps);
}

/* 作用：执行 3000 SPS 正常循迹，并在 14000 STEP 后减速。 */
static void MotorTrack_TaskLineFast(void)
{
    MotorTrack_RunLineState(CAR_MOTOR_TRACK_FAST_SPEED_SPS);
    if (g_motorTrack.state != MOTOR_TRACK_STATE_LINE_FAST) {
        return;
    }

    if (MotorTrack_UpdateTravelSteps() >=
        (uint32_t)CAR_MOTOR_TRACK_DECEL_STEPS) {
        g_motorTrack.baseSpeedSps = CAR_MOTOR_TRACK_SLOW_SPEED_SPS;
        g_motorTrack.state = MOTOR_TRACK_STATE_LINE_SLOW;
        LOG_LINE("[MOTOR TRACK] slow");
    }
}

/* 作用：执行 2000 SPS 低速循迹，只理会右转灰度信号。 */
static void MotorTrack_TaskLineSlow(void)
{
    MotorTrack_RunLineState(CAR_MOTOR_TRACK_SLOW_SPEED_SPS);
    if (g_motorTrack.state != MOTOR_TRACK_STATE_LINE_SLOW) {
        return;
    }

    (void)MotorTrack_UpdateTravelSteps();
    if (MotorTrack_IsRightTurnDetected(g_motorTrack.digitalMask) != 0U) {
        MotorTrack_StartRightTurn();
    }
}

/* 作用：左轮前进、右轮后退等速强右转，并用 yaw 判断是否转够。 */
static void MotorTrack_TaskTurningSpin(void)
{
    int16_t yawDelta;
    uint16_t timeoutTicks =
        MotorTrack_MsToTicks(CAR_MOTOR_TRACK_TURN_TIMEOUT_MS);

    Motion_SetChassisCommand((int16_t)CAR_MOTOR_TRACK_SLOW_SPEED_SPS,
        (int16_t)(-(int32_t)CAR_MOTOR_TRACK_SLOW_SPEED_SPS));

    yawDelta = MotorTrack_NormalizeYawDelta(JY61P_GetYawDeg(),
        g_motorTrack.yawStartDeg);
    g_motorTrack.yawDeltaDeg = MotorTrack_Abs16(yawDelta);
    MotorTrack_OnTurnYawDelta(g_motorTrack.yawDeltaDeg);

    if (g_motorTrack.yawDeltaDeg >
        (int16_t)CAR_MOTOR_TRACK_TURN_YAW_DEG) {
        g_motorTrack.exitSlowTicks = 0U;
        g_motorTrack.state = MOTOR_TRACK_STATE_TURN_EXIT_SLOW;
        LOG_RAW("[MOTOR TRACK] turn ok yaw=");
        LogUart_SendSigned(g_motorTrack.yawDeltaDeg);
        LOG_LINE("");
        return;
    }

    if (g_motorTrack.turnTicks >= timeoutTicks) {
        MotorTrack_StopWithReason("turn-timeout");
        return;
    }
    ++g_motorTrack.turnTicks;
}

/* 作用：yaw 达标后继续低速向前 200ms，再恢复高速并重新计 STEP。 */
static void MotorTrack_TaskTurnExitSlow(void)
{
    uint16_t exitTicks = MotorTrack_MsToTicks(CAR_MOTOR_TRACK_EXIT_SLOW_MS);

    Motion_Forward(CAR_MOTOR_TRACK_SLOW_SPEED_SPS);
    if (g_motorTrack.exitSlowTicks >= exitTicks) {
        g_motorTrack.phase = (uint8_t)((g_motorTrack.phase + 1U) & 0x03U);
        MotorTrack_OnPhaseChanged(g_motorTrack.phase);
        MotorTrack_ResetSegmentSteps();
        g_motorTrack.baseSpeedSps = CAR_MOTOR_TRACK_FAST_SPEED_SPS;
        g_motorTrack.state = MOTOR_TRACK_STATE_LINE_FAST;
        LOG_LINE("[MOTOR TRACK] fast");
        return;
    }
    ++g_motorTrack.exitSlowTicks;
}

void MotorTrack_Init(void)
{
    g_motorTrack.leftBaseStep = 0;
    g_motorTrack.rightBaseStep = 0;
    g_motorTrack.travelSteps = 0U;
    g_motorTrack.baseSpeedSps = CAR_MOTOR_TRACK_FAST_SPEED_SPS;
    g_motorTrack.turnTicks = 0U;
    g_motorTrack.exitSlowTicks = 0U;
    g_motorTrack.lineError = 0;
    g_motorTrack.yawStartDeg = 0;
    g_motorTrack.yawDeltaDeg = 0;
    g_motorTrack.phase = 0U;
    g_motorTrack.digitalMask = 0U;
    g_motorTrack.running = 0U;
    g_motorTrack.state = MOTOR_TRACK_STATE_IDLE;
}

void MotorTrack_Start(void)
{
    Motor_ResetStepCount(MOTOR_CHASSIS_LEFT);
    Motor_ResetStepCount(MOTOR_CHASSIS_RIGHT);
    MotorTrack_ResetSegmentSteps();
    g_motorTrack.baseSpeedSps = CAR_MOTOR_TRACK_FAST_SPEED_SPS;
    g_motorTrack.turnTicks = 0U;
    g_motorTrack.exitSlowTicks = 0U;
    g_motorTrack.lineError = 0;
    g_motorTrack.yawStartDeg = 0;
    g_motorTrack.yawDeltaDeg = 0;
    g_motorTrack.phase = 0U;
    g_motorTrack.digitalMask = 0U;
    g_motorTrack.running = 1U;
    g_motorTrack.state = MOTOR_TRACK_STATE_LINE_FAST;
    MotorEnable_SetChassis(1U);
    LOG_LINE("[MOTOR TRACK] start");
}

void MotorTrack_Stop(void)
{
    if (g_motorTrack.state != MOTOR_TRACK_STATE_IDLE) {
        Motion_SetChassisCommand(0, 0);
        MotorEnable_SetChassis(CAR_STEPPER_ENABLE_DEFAULT_ON);
    }
    g_motorTrack.running = 0U;
    g_motorTrack.state = MOTOR_TRACK_STATE_IDLE;
}

void MotorTrack_Task(void)
{
    if (g_motorTrack.running == 0U) {
        return;
    }

    switch (g_motorTrack.state) {
    case MOTOR_TRACK_STATE_LINE_FAST:
        MotorTrack_TaskLineFast();
        break;
    case MOTOR_TRACK_STATE_LINE_SLOW:
        MotorTrack_TaskLineSlow();
        break;
    case MOTOR_TRACK_STATE_TURNING_SPIN:
        MotorTrack_TaskTurningSpin();
        break;
    case MOTOR_TRACK_STATE_TURN_EXIT_SLOW:
        MotorTrack_TaskTurnExitSlow();
        break;
    case MOTOR_TRACK_STATE_STOP:
    case MOTOR_TRACK_STATE_IDLE:
    default:
        break;
    }
}

uint8_t MotorTrack_IsRunning(void)
{
    return g_motorTrack.running;
}

MotorTrackState MotorTrack_GetState(void)
{
    return g_motorTrack.state;
}

const char *MotorTrack_GetStateName(void)
{
    switch (g_motorTrack.state) {
    case MOTOR_TRACK_STATE_LINE_FAST:
        return "Fast";
    case MOTOR_TRACK_STATE_LINE_SLOW:
        return "Slow";
    case MOTOR_TRACK_STATE_TURNING_SPIN:
        return "Turn";
    case MOTOR_TRACK_STATE_TURN_EXIT_SLOW:
        return "Exit";
    case MOTOR_TRACK_STATE_STOP:
        return "Stop";
    case MOTOR_TRACK_STATE_IDLE:
    default:
        return "Idle";
    }
}

uint8_t MotorTrack_GetPhase(void)
{
    return g_motorTrack.phase;
}

uint32_t MotorTrack_GetTravelSteps(void)
{
    return g_motorTrack.travelSteps;
}

int16_t MotorTrack_GetYawDeltaDeg(void)
{
    return g_motorTrack.yawDeltaDeg;
}

uint8_t MotorTrack_IsTurning(void)
{
    return ((g_motorTrack.state == MOTOR_TRACK_STATE_TURNING_SPIN) ||
        (g_motorTrack.state == MOTOR_TRACK_STATE_TURN_EXIT_SLOW)) ? 1U : 0U;
}
