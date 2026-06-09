#include "gimbal_motor_test.h"

#include "board_config.h"
#include "gimbal.h"
#include "log_uart.h"
#include "motor.h"
#include "motor_enable.h"

typedef struct {
    GimbalMotorTestStage stage;
    uint16_t yawSpeedSps;
    uint16_t pitchSpeedSps;
    uint8_t running;
} GimbalMotorTestState;

static GimbalMotorTestState g_gimbalMotorTest;

/* 作用：把菜单调速值限制在统一的步进速度范围内。 */
static uint16_t GimbalMotorTest_ClampSpeed(uint32_t speedSps)
{
    if (speedSps < CAR_STEPPER_SPEED_MIN_SPS) {
        return CAR_STEPPER_SPEED_MIN_SPS;
    }
    if (speedSps > CAR_STEPPER_SPEED_MAX_SPS) {
        return CAR_STEPPER_SPEED_MAX_SPS;
    }
    return (uint16_t)speedSps;
}

/* 作用：读取测试方向配置，便于实车方向相反时统一反相。 */
static MotorDir GimbalMotorTest_GetDirection(void)
{
    return (CAR_GIMBAL_MOTOR_TEST_REVERSE != 0U) ?
        MOTOR_REVERSE : MOTOR_FORWARD;
}

/*
 * 作用：只停止云台两个轴。
 * 说明：不调用 Motor_Stop，避免误停或清理底盘状态。
 */
static void GimbalMotorTest_StopAxes(void)
{
    Gimbal_SetEnabled(0U);
    Motor_Set(MOTOR_GIMBAL_1, MOTOR_COAST, 0U);
    Motor_Set(MOTOR_GIMBAL_2, MOTOR_COAST, 0U);
}

/* 作用：把当前 yaw/pitch SPS 写到云台两个轴。 */
static void GimbalMotorTest_ApplySpeed(void)
{
    if (g_gimbalMotorTest.running == 0U) {
        return;
    }

    Motor_Set(MOTOR_GIMBAL_1, GimbalMotorTest_GetDirection(),
        g_gimbalMotorTest.yawSpeedSps);
    Motor_Set(MOTOR_GIMBAL_2, GimbalMotorTest_GetDirection(),
        g_gimbalMotorTest.pitchSpeedSps);
}

/* 作用：初始化云台电机 SPS 测试状态，默认不输出速度。 */
void GimbalMotorTest_Init(void)
{
    g_gimbalMotorTest.stage = GIMBAL_MOTOR_TEST_STAGE_IDLE;
    g_gimbalMotorTest.yawSpeedSps = CAR_GIMBAL_TEST_YAW_SPEED_SPS;
    g_gimbalMotorTest.pitchSpeedSps = CAR_GIMBAL_TEST_PITCH_SPEED_SPS;
    g_gimbalMotorTest.running = 0U;
}

/*
 * 作用：开始云台电机 SPS 测试。
 * 使用场景：菜单 Gimbal 进入时调用。
 * 说明：yaw 默认 2500 SPS，pitch 默认 1500 SPS，K1/K2 可同时按 500 SPS 微调。
 */
void GimbalMotorTest_Start(void)
{
    Motor_ResetStepCount(MOTOR_GIMBAL_1);
    Motor_ResetStepCount(MOTOR_GIMBAL_2);
    g_gimbalMotorTest.stage = GIMBAL_MOTOR_TEST_STAGE_SPEED;
    g_gimbalMotorTest.yawSpeedSps =
        GimbalMotorTest_ClampSpeed(CAR_GIMBAL_TEST_YAW_SPEED_SPS);
    g_gimbalMotorTest.pitchSpeedSps =
        GimbalMotorTest_ClampSpeed(CAR_GIMBAL_TEST_PITCH_SPEED_SPS);
    g_gimbalMotorTest.running = 1U;

    MotorEnable_SetGimbal(1U);
    GimbalMotorTest_ApplySpeed();

    LOG_RAW("gimbal test: yaw=");
    LogUart_SendUnsigned(g_gimbalMotorTest.yawSpeedSps);
    LOG_RAW(" pitch=");
    LogUart_SendUnsigned(g_gimbalMotorTest.pitchSpeedSps);
    LOG_LINE("");
}

/* 作用：停止云台电机 SPS 测试并恢复默认 EN 状态。 */
void GimbalMotorTest_Stop(void)
{
    if (g_gimbalMotorTest.running != 0U) {
        GimbalMotorTest_StopAxes();
        MotorEnable_SetGimbal(CAR_STEPPER_ENABLE_DEFAULT_ON);
    }
    g_gimbalMotorTest.stage = GIMBAL_MOTOR_TEST_STAGE_IDLE;
    g_gimbalMotorTest.running = 0U;
}

/*
 * 作用：保持当前 SPS 输出。
 * 使用场景：状态机处于 Gimbal 测试时每轮调用。
 */
void GimbalMotorTest_Task(void)
{
    GimbalMotorTest_ApplySpeed();
}

/* 作用：按配置步长提高云台测试速度。 */
void GimbalMotorTest_IncreaseSpeed(void)
{
    g_gimbalMotorTest.yawSpeedSps = GimbalMotorTest_ClampSpeed(
        (uint32_t)g_gimbalMotorTest.yawSpeedSps +
            CAR_STEPPER_SPEED_STEP_SPS);
    g_gimbalMotorTest.pitchSpeedSps = GimbalMotorTest_ClampSpeed(
        (uint32_t)g_gimbalMotorTest.pitchSpeedSps +
            CAR_STEPPER_SPEED_STEP_SPS);
    GimbalMotorTest_ApplySpeed();
}

/* 作用：按配置步长降低云台测试速度。 */
void GimbalMotorTest_DecreaseSpeed(void)
{
    uint32_t yawSpeedSps = 0U;
    uint32_t pitchSpeedSps = 0U;

    if (g_gimbalMotorTest.yawSpeedSps > CAR_STEPPER_SPEED_STEP_SPS) {
        yawSpeedSps = (uint32_t)g_gimbalMotorTest.yawSpeedSps -
            CAR_STEPPER_SPEED_STEP_SPS;
    }
    if (g_gimbalMotorTest.pitchSpeedSps > CAR_STEPPER_SPEED_STEP_SPS) {
        pitchSpeedSps = (uint32_t)g_gimbalMotorTest.pitchSpeedSps -
            CAR_STEPPER_SPEED_STEP_SPS;
    }
    g_gimbalMotorTest.yawSpeedSps =
        GimbalMotorTest_ClampSpeed(yawSpeedSps);
    g_gimbalMotorTest.pitchSpeedSps =
        GimbalMotorTest_ClampSpeed(pitchSpeedSps);
    GimbalMotorTest_ApplySpeed();
}

/* 作用：兼容旧接口，返回 yaw 当前测试速度，单位 step/s。 */
uint16_t GimbalMotorTest_GetSpeedSps(void)
{
    return g_gimbalMotorTest.yawSpeedSps;
}

/* 作用：读取 yaw 当前测试速度，单位 step/s。 */
uint16_t GimbalMotorTest_GetYawSpeedSps(void)
{
    return g_gimbalMotorTest.yawSpeedSps;
}

/* 作用：读取 pitch 当前测试速度，单位 step/s。 */
uint16_t GimbalMotorTest_GetPitchSpeedSps(void)
{
    return g_gimbalMotorTest.pitchSpeedSps;
}

/* 作用：返回云台电机测试是否正在输出速度。 */
uint8_t GimbalMotorTest_IsRunning(void)
{
    return g_gimbalMotorTest.running;
}

/* 作用：返回当前测试阶段枚举。 */
GimbalMotorTestStage GimbalMotorTest_GetStage(void)
{
    return g_gimbalMotorTest.stage;
}

/* 作用：返回当前测试阶段名称，用于 OLED 和日志。 */
const char *GimbalMotorTest_GetStageName(void)
{
    switch (g_gimbalMotorTest.stage) {
    case GIMBAL_MOTOR_TEST_STAGE_SPEED:
        return "Speed";
    case GIMBAL_MOTOR_TEST_STAGE_IDLE:
    default:
        return "Idle";
    }
}

/* 作用：兼容旧显示接口；当前持续调速模式不计算百分比。 */
uint8_t GimbalMotorTest_GetProgressPercent(void)
{
    return 0U;
}
