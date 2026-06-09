#include "track_step_test.h"

#include "board_config.h"
#include "gray.h"
#include "log_uart.h"
#include "motion.h"
#include "motor.h"
#include "motor_enable.h"

#define TRACK_STEP_TEST_LEFT_MASK       (0x06U)
#define TRACK_STEP_TEST_CENTER_MASK     (0x08U)
#define TRACK_STEP_TEST_RIGHT_MASK      (0x30U)
#define TRACK_STEP_TEST_TURN_EXIT_ERROR (GRAY_LINE_ERROR_SCALE)

typedef enum {
    TRACK_STEP_TEST_MODE_IDLE = 0,
    TRACK_STEP_TEST_MODE_FOLLOW,
    TRACK_STEP_TEST_MODE_TURN_LEFT,
    TRACK_STEP_TEST_MODE_TURN_RIGHT,
    TRACK_STEP_TEST_MODE_LOST_LEFT,
    TRACK_STEP_TEST_MODE_LOST_RIGHT,
    TRACK_STEP_TEST_MODE_GRAY_FAULT
} TrackStepTestMode;

typedef struct {
    int32_t leftBaseStep;
    int32_t rightBaseStep;
    uint16_t baseSpeedSps;
    uint16_t turnSpeedSps;
    int16_t lineError;
    int8_t lastLineSide;
    TrackStepTestMode mode;
    uint8_t digitalMask;
    uint8_t running;
} TrackStepTestState;

static TrackStepTestState g_trackStepTest;

/* 作用：计算 int32_t 绝对值，用于把正反向 STEP 都转成路程。 */
static uint32_t TrackStepTest_Abs32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

/* 作用：计算 int16_t 绝对值，用于判断急转是否回到中心附近。 */
static uint16_t TrackStepTest_Abs16(int16_t value)
{
    return (value < 0) ? (uint16_t)(-(int32_t)value) : (uint16_t)value;
}

/* 作用：把菜单调速值限制在统一的步进速度范围内。 */
static uint16_t TrackStepTest_ClampSpeed(uint32_t speedSps)
{
    if (speedSps < CAR_STEPPER_SPEED_MIN_SPS) {
        return CAR_STEPPER_SPEED_MIN_SPS;
    }
    if (speedSps > CAR_STEPPER_SPEED_MAX_SPS) {
        return CAR_STEPPER_SPEED_MAX_SPS;
    }
    return (uint16_t)speedSps;
}

/* 作用：限制普通循迹差速修正，避免基础速度被修正量完全吃掉。 */
static int16_t TrackStepTest_ClampCorrection(int32_t correction,
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

/*
 * 作用：让底盘原地左转。
 * 说明：左轮是内轮，DIR 反向；右轮是外轮，保持正向。
 */
static void TrackStepTest_TurnLeft(TrackStepTestMode mode)
{
    g_trackStepTest.mode = mode;
    Motion_SetChassisCommand((int16_t)(-(int32_t)g_trackStepTest.turnSpeedSps),
        (int16_t)g_trackStepTest.turnSpeedSps);
}

/*
 * 作用：让底盘原地右转。
 * 说明：右轮是内轮，DIR 反向；左轮是外轮，保持正向。
 */
static void TrackStepTest_TurnRight(TrackStepTestMode mode)
{
    g_trackStepTest.mode = mode;
    Motion_SetChassisCommand((int16_t)g_trackStepTest.turnSpeedSps,
        (int16_t)(-(int32_t)g_trackStepTest.turnSpeedSps));
}

/* 作用：丢线时按最近一次偏向继续原地找线。 */
static void TrackStepTest_SearchLostLine(void)
{
    if (g_trackStepTest.lastLineSide > 0) {
        TrackStepTest_TurnRight(TRACK_STEP_TEST_MODE_LOST_RIGHT);
    } else {
        TrackStepTest_TurnLeft(TRACK_STEP_TEST_MODE_LOST_LEFT);
    }
}

/* 作用：判断当前是否处在侧边触发后的锁存急转。 */
static uint8_t TrackStepTest_IsLatchedTurn(void)
{
    return ((g_trackStepTest.mode == TRACK_STEP_TEST_MODE_TURN_LEFT) ||
        (g_trackStepTest.mode == TRACK_STEP_TEST_MODE_TURN_RIGHT)) ? 1U : 0U;
}

/* 作用：锁存急转未结束时继续保持内轮反转、外轮正转。 */
static void TrackStepTest_ContinueLatchedTurn(void)
{
    if (g_trackStepTest.mode == TRACK_STEP_TEST_MODE_TURN_RIGHT) {
        TrackStepTest_TurnRight(TRACK_STEP_TEST_MODE_TURN_RIGHT);
    } else {
        TrackStepTest_TurnLeft(TRACK_STEP_TEST_MODE_TURN_LEFT);
    }
}

/*
 * 作用：判断锁存急转是否可以释放。
 * 说明：中心路重新压线，或加权误差回到中心附近，就切回普通 Follow。
 */
static uint8_t TrackStepTest_ShouldExitLatchedTurn(uint8_t digitalMask,
    int16_t lineError)
{
    if ((digitalMask & TRACK_STEP_TEST_CENTER_MASK) != 0U) {
        return 1U;
    }

    return (TrackStepTest_Abs16(lineError) <=
        (uint16_t)TRACK_STEP_TEST_TURN_EXIT_ERROR) ? 1U : 0U;
}

/* 作用：按当前加权灰度误差生成普通差速循迹速度。 */
static void TrackStepTest_FollowLine(void)
{
    int16_t baseSpeed = (int16_t)g_trackStepTest.baseSpeedSps;
    uint16_t correctionLimit =
        (g_trackStepTest.turnSpeedSps < g_trackStepTest.baseSpeedSps) ?
            g_trackStepTest.turnSpeedSps : g_trackStepTest.baseSpeedSps;
    int16_t correction = TrackStepTest_ClampCorrection(
        ((int32_t)g_trackStepTest.lineError * (int32_t)CAR_TRACK_TURN_GAIN) /
            (int32_t)GRAY_LINE_ERROR_SCALE,
        correctionLimit);

    g_trackStepTest.mode = TRACK_STEP_TEST_MODE_FOLLOW;
    Motion_SetChassisCommand((int16_t)(baseSpeed + correction),
        (int16_t)(baseSpeed - correction));
}

/*
 * 作用：执行一轮 Motor 菜单里的无限循迹测试。
 * 说明：S2/S3 或 S5/S6 同侧全压线时按急转处理；普通位置用加权误差差速跟线。
 */
static void TrackStepTest_RunLineControl(void)
{
    uint8_t leftAllActive;
    uint8_t rightAllActive;
    uint8_t hasLineError = 0U;
    int16_t lineError = 0;

    if (g_trackStepTest.running == 0U) {
        return;
    }

    if (!Gray_Update()) {
        g_trackStepTest.mode = TRACK_STEP_TEST_MODE_GRAY_FAULT;
        Motion_SetChassisCommand(0, 0);
        return;
    }

    g_trackStepTest.digitalMask = Gray_GetDigitalMask();
    if (g_trackStepTest.digitalMask == 0U) {
        if (TrackStepTest_IsLatchedTurn() != 0U) {
            TrackStepTest_ContinueLatchedTurn();
            return;
        }
        TrackStepTest_SearchLostLine();
        return;
    }

    hasLineError = Gray_GetWeightedLineError(&lineError);
    if (hasLineError != 0U) {
        g_trackStepTest.lineError = lineError;
    } else if ((g_trackStepTest.digitalMask & TRACK_STEP_TEST_CENTER_MASK) !=
        0U) {
        g_trackStepTest.lineError = 0;
    }

    if (TrackStepTest_IsLatchedTurn() != 0U) {
        if ((hasLineError != 0U) &&
            TrackStepTest_ShouldExitLatchedTurn(g_trackStepTest.digitalMask,
                lineError)) {
            TrackStepTest_FollowLine();
        } else {
            TrackStepTest_ContinueLatchedTurn();
        }
        return;
    }

    leftAllActive =
        ((g_trackStepTest.digitalMask & TRACK_STEP_TEST_LEFT_MASK) ==
            TRACK_STEP_TEST_LEFT_MASK) ? 1U : 0U;
    rightAllActive =
        ((g_trackStepTest.digitalMask & TRACK_STEP_TEST_RIGHT_MASK) ==
            TRACK_STEP_TEST_RIGHT_MASK) ? 1U : 0U;

    if ((leftAllActive != 0U) && (rightAllActive == 0U)) {
        g_trackStepTest.lastLineSide = -1;
        TrackStepTest_TurnLeft(TRACK_STEP_TEST_MODE_TURN_LEFT);
        return;
    }
    if ((rightAllActive != 0U) && (leftAllActive == 0U)) {
        g_trackStepTest.lastLineSide = 1;
        TrackStepTest_TurnRight(TRACK_STEP_TEST_MODE_TURN_RIGHT);
        return;
    }

    if (hasLineError != 0U) {
        if (g_trackStepTest.lineError < 0) {
            g_trackStepTest.lastLineSide = -1;
        } else if (g_trackStepTest.lineError > 0) {
            g_trackStepTest.lastLineSide = 1;
        }
    }

    TrackStepTest_FollowLine();
}

/*
 * 作用：读取底盘左右轮相对测试起点的平均 STEP 数。
 * 说明：这里统计的是 MCU 已输出脉冲数，不代表驱动器真实反馈。
 */
uint32_t TrackStepTest_GetTravelSteps(void)
{
    int32_t leftDelta =
        Motor_GetStepCount(MOTOR_CHASSIS_LEFT) - g_trackStepTest.leftBaseStep;
    int32_t rightDelta =
        Motor_GetStepCount(MOTOR_CHASSIS_RIGHT) - g_trackStepTest.rightBaseStep;

    return (TrackStepTest_Abs32(leftDelta) +
        TrackStepTest_Abs32(rightDelta)) / 2U;
}

/* 作用：初始化底盘电机 SPS 测试状态，默认不输出电机速度。 */
void TrackStepTest_Init(void)
{
    g_trackStepTest.leftBaseStep = 0;
    g_trackStepTest.rightBaseStep = 0;
    g_trackStepTest.baseSpeedSps = CAR_MOTOR_TEST_DEFAULT_SPEED_SPS;
    g_trackStepTest.turnSpeedSps = CAR_MOTOR_TEST_TURN_SPEED_SPS;
    g_trackStepTest.lineError = 0;
    g_trackStepTest.lastLineSide =
        CAR_TRACK_SEARCH_DEFAULT_LEFT ? -1 : 1;
    g_trackStepTest.mode = TRACK_STEP_TEST_MODE_IDLE;
    g_trackStepTest.digitalMask = 0U;
    g_trackStepTest.running = 0U;
}

/*
 * 作用：开始底盘无限循迹测试。
 * 使用场景：菜单 Motor 进入时调用。
 * 说明：基础速度默认 3000 SPS，K1/K2 可按 500 SPS 调整基础速度。
 */
void TrackStepTest_Start(void)
{
    Motor_ResetStepCount(MOTOR_CHASSIS_LEFT);
    Motor_ResetStepCount(MOTOR_CHASSIS_RIGHT);
    g_trackStepTest.leftBaseStep = Motor_GetStepCount(MOTOR_CHASSIS_LEFT);
    g_trackStepTest.rightBaseStep = Motor_GetStepCount(MOTOR_CHASSIS_RIGHT);
    g_trackStepTest.baseSpeedSps =
        TrackStepTest_ClampSpeed(CAR_MOTOR_TEST_DEFAULT_SPEED_SPS);
    g_trackStepTest.turnSpeedSps =
        TrackStepTest_ClampSpeed(CAR_MOTOR_TEST_TURN_SPEED_SPS);
    g_trackStepTest.lineError = 0;
    g_trackStepTest.lastLineSide =
        CAR_TRACK_SEARCH_DEFAULT_LEFT ? -1 : 1;
    g_trackStepTest.mode = TRACK_STEP_TEST_MODE_FOLLOW;
    g_trackStepTest.digitalMask = 0U;
    g_trackStepTest.running = 1U;

    MotorEnable_SetAll(1U);
    TrackStepTest_RunLineControl();

    LOG_RAW("motor track test: base=");
    LogUart_SendUnsigned(g_trackStepTest.baseSpeedSps);
    LOG_RAW(" turn=");
    LogUart_SendUnsigned(g_trackStepTest.turnSpeedSps);
    LOG_LINE("");
}

/* 作用：停止底盘无限循迹测试，只清底盘速度并恢复默认 EN 状态。 */
void TrackStepTest_Stop(void)
{
    if (g_trackStepTest.running != 0U) {
        Motion_SetChassisCommand(0, 0);
        MotorEnable_SetAll(CAR_STEPPER_ENABLE_DEFAULT_ON);
    }
    g_trackStepTest.mode = TRACK_STEP_TEST_MODE_IDLE;
    g_trackStepTest.running = 0U;
}

/*
 * 作用：周期执行灰度无限循迹。
 * 说明：STEP 仍由 TIMG0 中断输出，本函数只更新左右轮目标 SPS 和方向。
 */
void TrackStepTest_Task(void)
{
    TrackStepTest_RunLineControl();
}

/* 作用：按配置步长提高底盘测试基础速度。 */
void TrackStepTest_IncreaseSpeed(void)
{
    g_trackStepTest.baseSpeedSps = TrackStepTest_ClampSpeed(
        (uint32_t)g_trackStepTest.baseSpeedSps + CAR_STEPPER_SPEED_STEP_SPS);
    TrackStepTest_RunLineControl();
}

/* 作用：按配置步长降低底盘测试基础速度。 */
void TrackStepTest_DecreaseSpeed(void)
{
    uint32_t speedSps = 0U;

    if (g_trackStepTest.baseSpeedSps > CAR_STEPPER_SPEED_STEP_SPS) {
        speedSps = (uint32_t)g_trackStepTest.baseSpeedSps -
            CAR_STEPPER_SPEED_STEP_SPS;
    }
    g_trackStepTest.baseSpeedSps = TrackStepTest_ClampSpeed(speedSps);
    TrackStepTest_RunLineControl();
}

/* 作用：读取当前底盘测试基础速度，单位 step/s。 */
uint16_t TrackStepTest_GetSpeedSps(void)
{
    return g_trackStepTest.baseSpeedSps;
}

/* 作用：读取急转时内外轮使用的速度，单位 step/s。 */
uint16_t TrackStepTest_GetTurnSpeedSps(void)
{
    return g_trackStepTest.turnSpeedSps;
}

/* 作用：读取当前循迹模式名称，用于 OLED 状态显示。 */
const char *TrackStepTest_GetModeName(void)
{
    switch (g_trackStepTest.mode) {
    case TRACK_STEP_TEST_MODE_FOLLOW:
        return "Follow";
    case TRACK_STEP_TEST_MODE_TURN_LEFT:
        return "Turn Left";
    case TRACK_STEP_TEST_MODE_TURN_RIGHT:
        return "Turn Right";
    case TRACK_STEP_TEST_MODE_LOST_LEFT:
        return "Lost Left";
    case TRACK_STEP_TEST_MODE_LOST_RIGHT:
        return "Lost Right";
    case TRACK_STEP_TEST_MODE_GRAY_FAULT:
        return "Gray Fault";
    case TRACK_STEP_TEST_MODE_IDLE:
    default:
        return "Idle";
    }
}

/* 作用：返回底盘测试是否正在输出速度。 */
uint8_t TrackStepTest_IsRunning(void)
{
    return g_trackStepTest.running;
}

/* 作用：兼容旧显示接口；当前持续调速模式没有自动完成状态。 */
uint8_t TrackStepTest_IsDone(void)
{
    return 0U;
}

/* 作用：兼容旧显示接口；当前持续调速模式不计算百分比。 */
uint8_t TrackStepTest_GetProgressPercent(void)
{
    return 0U;
}
