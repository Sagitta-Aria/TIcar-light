#include "track_step_test.h"

#include "board_config.h"
#include "log_uart.h"
#include "motion.h"
#include "motor.h"
#include "motor_enable.h"

typedef enum {
    TRACK_STEP_TEST_MODE_IDLE = 0,
    TRACK_STEP_TEST_MODE_RUN,
    TRACK_STEP_TEST_MODE_DONE
} TrackStepTestMode;

typedef struct {
    int32_t leftBaseStep;
    int32_t rightBaseStep;
    uint16_t baseSpeedSps;
    TrackStepTestMode mode;
    uint8_t running;
} TrackStepTestState;

static TrackStepTestState g_trackStepTest;

/* 作用：计算 int32_t 绝对值，用于把正反向 STEP 都转成路程。 */
static uint32_t TrackStepTest_Abs32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
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

/* 作用：停止底盘输出并恢复底盘 EN 默认状态。 */
static void TrackStepTest_StopChassis(void)
{
    Motion_SetChassisCommand(0, 0);
    MotorEnable_SetChassis(CAR_STEPPER_ENABLE_DEFAULT_ON);
}

/* 作用：按当前基础速度持续输出前进命令。 */
static void TrackStepTest_ApplyRunSpeed(void)
{
    if (g_trackStepTest.running != 0U) {
        Motion_Forward(g_trackStepTest.baseSpeedSps);
    }
}

/*
 * 作用：检查固定脉冲测试是否完成。
 * 说明：左右轮平均 STEP 达到目标后立即停车；不读取灰度传感器。
 */
static void TrackStepTest_UpdateFixedStepRun(void)
{
    uint32_t travelSteps;

    if (g_trackStepTest.running == 0U) {
        return;
    }

    travelSteps = TrackStepTest_GetTravelSteps();
    if (travelSteps >= (uint32_t)CAR_TRACK_STEP_TEST_TARGET_STEPS) {
        TrackStepTest_StopChassis();
        g_trackStepTest.running = 0U;
        g_trackStepTest.mode = TRACK_STEP_TEST_MODE_DONE;
        LOG_RAW("motor step test: done steps=");
        LogUart_SendUnsigned(travelSteps);
        LOG_LINE("");
        return;
    }

    TrackStepTest_ApplyRunSpeed();
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

/* 作用：初始化底盘固定脉冲测试状态，默认不输出电机速度。 */
void TrackStepTest_Init(void)
{
    g_trackStepTest.leftBaseStep = 0;
    g_trackStepTest.rightBaseStep = 0;
    g_trackStepTest.baseSpeedSps = CAR_MOTOR_TEST_DEFAULT_SPEED_SPS;
    g_trackStepTest.mode = TRACK_STEP_TEST_MODE_IDLE;
    g_trackStepTest.running = 0U;
}

/*
 * 作用：开始底盘固定脉冲测试。
 * 使用场景：菜单 Motor 进入时调用。
 * 说明：当前没有灰度传感器，所以上电后只前进 7835 个平均 STEP 并停止。
 */
void TrackStepTest_Start(void)
{
    Motor_ResetStepCount(MOTOR_CHASSIS_LEFT);
    Motor_ResetStepCount(MOTOR_CHASSIS_RIGHT);
    g_trackStepTest.leftBaseStep = Motor_GetStepCount(MOTOR_CHASSIS_LEFT);
    g_trackStepTest.rightBaseStep = Motor_GetStepCount(MOTOR_CHASSIS_RIGHT);
    g_trackStepTest.baseSpeedSps =
        TrackStepTest_ClampSpeed(CAR_MOTOR_TEST_DEFAULT_SPEED_SPS);
    g_trackStepTest.mode = TRACK_STEP_TEST_MODE_RUN;
    g_trackStepTest.running = 1U;

    MotorEnable_SetChassis(1U);
    TrackStepTest_ApplyRunSpeed();

    LOG_RAW("motor step test: speed=");
    LogUart_SendUnsigned(g_trackStepTest.baseSpeedSps);
    LOG_RAW(" target=");
    LogUart_SendUnsigned(CAR_TRACK_STEP_TEST_TARGET_STEPS);
    LOG_LINE("");
}

/* 作用：停止底盘固定脉冲测试，只清底盘速度并恢复默认 EN 状态。 */
void TrackStepTest_Stop(void)
{
    if (g_trackStepTest.running != 0U) {
        TrackStepTest_StopChassis();
    }
    g_trackStepTest.mode = TRACK_STEP_TEST_MODE_IDLE;
    g_trackStepTest.running = 0U;
}

/*
 * 作用：周期执行固定脉冲底盘测试。
 * 说明：STEP 仍由 TIMG0 中断输出，本函数只检查目标脉冲并保持速度。
 */
void TrackStepTest_Task(void)
{
    TrackStepTest_UpdateFixedStepRun();
}

/* 作用：按配置步长提高底盘测试基础速度。 */
void TrackStepTest_IncreaseSpeed(void)
{
    g_trackStepTest.baseSpeedSps = TrackStepTest_ClampSpeed(
        (uint32_t)g_trackStepTest.baseSpeedSps + CAR_STEPPER_SPEED_STEP_SPS);
    TrackStepTest_ApplyRunSpeed();
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
    TrackStepTest_ApplyRunSpeed();
}

/* 作用：读取当前底盘测试基础速度，单位 step/s。 */
uint16_t TrackStepTest_GetSpeedSps(void)
{
    return g_trackStepTest.baseSpeedSps;
}

/* 作用：兼容旧接口；当前固定脉冲测试没有单独急转速度。 */
uint16_t TrackStepTest_GetTurnSpeedSps(void)
{
    return 0U;
}

/* 作用：读取当前测试模式名称，用于 OLED 状态显示。 */
const char *TrackStepTest_GetModeName(void)
{
    switch (g_trackStepTest.mode) {
    case TRACK_STEP_TEST_MODE_RUN:
        return "Run";
    case TRACK_STEP_TEST_MODE_DONE:
        return "Done";
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

/* 作用：返回固定脉冲测试是否已到达目标 STEP。 */
uint8_t TrackStepTest_IsDone(void)
{
    return (g_trackStepTest.mode == TRACK_STEP_TEST_MODE_DONE) ? 1U : 0U;
}

/* 作用：返回固定脉冲测试进度百分比。 */
uint8_t TrackStepTest_GetProgressPercent(void)
{
    uint32_t progress =
        (TrackStepTest_GetTravelSteps() * 100U) /
        (uint32_t)CAR_TRACK_STEP_TEST_TARGET_STEPS;

    return (progress > 100U) ? 100U : (uint8_t)progress;
}
