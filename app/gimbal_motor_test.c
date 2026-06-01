#include "gimbal_motor_test.h"

#include "board_config.h"
#include "gimbal.h"
#include "log_uart.h"
#include "motor.h"

#if (CAR_GIMBAL_MOTOR_TEST_LR_STEPS_PER_90 == 0U)
#error "CAR_GIMBAL_MOTOR_TEST_LR_STEPS_PER_90 must be greater than 0"
#endif

#if (CAR_GIMBAL_MOTOR_TEST_UD_STEPS_PER_90 == 0U)
#error "CAR_GIMBAL_MOTOR_TEST_UD_STEPS_PER_90 must be greater than 0"
#endif

#if (CAR_GIMBAL_MOTOR_TEST_ZERO_TICKS == 0U)
#error "CAR_GIMBAL_MOTOR_TEST_ZERO_TICKS must be greater than 0"
#endif

#if (CAR_GIMBAL_MOTOR_TEST_SIM_HOLD_TICKS == 0U)
#error "CAR_GIMBAL_MOTOR_TEST_SIM_HOLD_TICKS must be greater than 0"
#endif

#if (CAR_GIMBAL_MOTOR_TEST_CIRCLE_PHASE_TICKS == 0U)
#error "CAR_GIMBAL_MOTOR_TEST_CIRCLE_PHASE_TICKS must be greater than 0"
#endif

#if (CAR_GIMBAL_MOTOR_TEST_CIRCLE_CYCLES == 0U)
#error "CAR_GIMBAL_MOTOR_TEST_CIRCLE_CYCLES must be greater than 0"
#endif

#define GIMBAL_MOTOR_TEST_CIRCLE_PHASE_COUNT (16U)

typedef struct {
    int16_t currentX;
    int16_t currentY;
} GimbalMotorTestVisionPoint;

typedef struct {
    GimbalMotorTestStage stage;
    int32_t axisBaseStep;
    uint16_t zeroTicks;
    uint32_t stageTicks;
    uint8_t running;
} GimbalMotorTestState;

static GimbalMotorTestState g_gimbalMotorTest;

/* 不接摄像头时，用这组假坐标模拟“激光点偏离目标点”。 */
static const GimbalMotorTestVisionPoint g_simVisionScript[] = {
    { (int16_t)(CAR_GIMBAL_TEST_TARGET_X - 1200),
      CAR_GIMBAL_TEST_TARGET_Y },
    { (int16_t)(CAR_GIMBAL_TEST_TARGET_X + 1200),
      CAR_GIMBAL_TEST_TARGET_Y },
    { CAR_GIMBAL_TEST_TARGET_X,
      (int16_t)(CAR_GIMBAL_TEST_TARGET_Y - 900) },
    { CAR_GIMBAL_TEST_TARGET_X,
      (int16_t)(CAR_GIMBAL_TEST_TARGET_Y + 900) },
    { (int16_t)(CAR_GIMBAL_TEST_TARGET_X - 850),
      (int16_t)(CAR_GIMBAL_TEST_TARGET_Y - 650) },
    { (int16_t)(CAR_GIMBAL_TEST_TARGET_X + 850),
      (int16_t)(CAR_GIMBAL_TEST_TARGET_Y - 650) },
    { (int16_t)(CAR_GIMBAL_TEST_TARGET_X + 850),
      (int16_t)(CAR_GIMBAL_TEST_TARGET_Y + 650) },
    { (int16_t)(CAR_GIMBAL_TEST_TARGET_X - 850),
      (int16_t)(CAR_GIMBAL_TEST_TARGET_Y + 650) },
    { CAR_GIMBAL_TEST_TARGET_X, CAR_GIMBAL_TEST_TARGET_Y }
};

/* 两轴速度相位表：按速度积分近似画圆，真实圆度后续靠视觉闭环修正。 */
static const int16_t g_circleVelocityTable[GIMBAL_MOTOR_TEST_CIRCLE_PHASE_COUNT][2] = {
    {     0,  1000 },
    {  -383,   924 },
    {  -707,   707 },
    {  -924,   383 },
    { -1000,     0 },
    {  -924,  -383 },
    {  -707,  -707 },
    {  -383,  -924 },
    {     0, -1000 },
    {   383,  -924 },
    {   707,  -707 },
    {   924,  -383 },
    {  1000,     0 },
    {   924,   383 },
    {   707,   707 },
    {   383,   924 }
};

const char *GimbalMotorTest_GetStageName(void);

static uint32_t GimbalMotorTest_Abs32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

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

static MotorId GimbalMotorTest_GetStageMotor(GimbalMotorTestStage stage)
{
    return (stage == GIMBAL_MOTOR_TEST_STAGE_UP_DOWN_90) ?
        MOTOR_GIMBAL_2 : MOTOR_GIMBAL_1;
}

static uint32_t GimbalMotorTest_GetStageTargetSteps(void)
{
    if (g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_UP_DOWN_90) {
        return CAR_GIMBAL_MOTOR_TEST_UD_STEPS_PER_90;
    }
    if (g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_LEFT_RIGHT_90) {
        return CAR_GIMBAL_MOTOR_TEST_LR_STEPS_PER_90;
    }
    return 1U;
}

static uint32_t GimbalMotorTest_GetScriptCount(void)
{
    return (uint32_t)(sizeof(g_simVisionScript) /
        sizeof(g_simVisionScript[0]));
}

static uint32_t GimbalMotorTest_GetSimTotalTicks(void)
{
    return GimbalMotorTest_GetScriptCount() *
        (uint32_t)CAR_GIMBAL_MOTOR_TEST_SIM_HOLD_TICKS;
}

static uint32_t GimbalMotorTest_GetCircleTotalTicks(void)
{
    return (uint32_t)GIMBAL_MOTOR_TEST_CIRCLE_PHASE_COUNT *
        (uint32_t)CAR_GIMBAL_MOTOR_TEST_CIRCLE_PHASE_TICKS *
        (uint32_t)CAR_GIMBAL_MOTOR_TEST_CIRCLE_CYCLES;
}

static void GimbalMotorTest_ApplySimVision(uint32_t scriptIndex)
{
    const GimbalMotorTestVisionPoint *point;

    if (scriptIndex >= GimbalMotorTest_GetScriptCount()) {
        scriptIndex = GimbalMotorTest_GetScriptCount() - 1U;
    }

    point = &g_simVisionScript[scriptIndex];
    Gimbal_UpdateFromVision(CAR_GIMBAL_TEST_TARGET_X,
        CAR_GIMBAL_TEST_TARGET_Y, point->currentX, point->currentY);
}

static int16_t GimbalMotorTest_ScaleCircleCommand(int16_t value)
{
    int32_t command = ((int32_t)value *
        (int32_t)CAR_GIMBAL_MOTOR_TEST_CIRCLE_COMMAND) / 1000;

    if (command > (int32_t)CAR_MOTOR_COMMAND_MAX) {
        command = (int32_t)CAR_MOTOR_COMMAND_MAX;
    }
    if (command < -(int32_t)CAR_MOTOR_COMMAND_MAX) {
        command = -(int32_t)CAR_MOTOR_COMMAND_MAX;
    }
    return (int16_t)command;
}

static void GimbalMotorTest_SetAxisCommand(MotorId motor, int16_t command)
{
    if (command > 0) {
        Motor_Set(motor, MOTOR_FORWARD, (uint16_t)command);
    } else if (command < 0) {
        Motor_Set(motor, MOTOR_REVERSE, (uint16_t)(-(int32_t)command));
    } else {
        Motor_Set(motor, MOTOR_COAST, 0U);
    }
}

static void GimbalMotorTest_ApplyCircle(uint32_t phase)
{
    int16_t commandX;
    int16_t commandY;

    phase %= GIMBAL_MOTOR_TEST_CIRCLE_PHASE_COUNT;
    commandX = GimbalMotorTest_ScaleCircleCommand(
        g_circleVelocityTable[phase][0]);
    commandY = GimbalMotorTest_ScaleCircleCommand(
        g_circleVelocityTable[phase][1]);

    GimbalMotorTest_SetAxisCommand(MOTOR_GIMBAL_1, commandX);
    GimbalMotorTest_SetAxisCommand(MOTOR_GIMBAL_2, commandY);
}

static void GimbalMotorTest_EnterStage(GimbalMotorTestStage stage)
{
    MotorId motor;

    GimbalMotorTest_StopAxes();
    g_gimbalMotorTest.stage = stage;
    g_gimbalMotorTest.zeroTicks = 0U;
    g_gimbalMotorTest.stageTicks = 0U;
    LOG_RAW("gimbal motor test: stage ");
    LOG_LINE(GimbalMotorTest_GetStageName());

    if (stage == GIMBAL_MOTOR_TEST_STAGE_ZERO) {
        Motor_ResetStepCount(MOTOR_GIMBAL_1);
        Motor_ResetStepCount(MOTOR_GIMBAL_2);
        g_gimbalMotorTest.axisBaseStep = 0;
        LOG_LINE("gimbal motor test: software zero");
        return;
    }

    if (stage == GIMBAL_MOTOR_TEST_STAGE_SIM_TRACK) {
        Gimbal_SetTarget(CAR_GIMBAL_TEST_TARGET_X, CAR_GIMBAL_TEST_TARGET_Y);
        GimbalMotorTest_ApplySimVision(0U);
        Gimbal_SetEnabled(1U);
        LOG_LINE("gimbal motor test: simulated vision script");
        return;
    }

    if (stage == GIMBAL_MOTOR_TEST_STAGE_CIRCLE) {
        GimbalMotorTest_ApplyCircle(0U);
        LOG_LINE("gimbal motor test: open loop circle");
        return;
    }

    if ((stage != GIMBAL_MOTOR_TEST_STAGE_LEFT_RIGHT_90) &&
        (stage != GIMBAL_MOTOR_TEST_STAGE_UP_DOWN_90)) {
        g_gimbalMotorTest.running = 0U;
        return;
    }

    motor = GimbalMotorTest_GetStageMotor(stage);
    g_gimbalMotorTest.axisBaseStep = Motor_GetStepCount(motor);
    Motor_Set(motor, GimbalMotorTest_GetDirection(),
        CAR_GIMBAL_MOTOR_TEST_COMMAND);
    LOG_RAW("gimbal motor test: command ");
    LogUart_SendUnsigned(CAR_GIMBAL_MOTOR_TEST_COMMAND);
    LOG_LINE("");
}

static uint32_t GimbalMotorTest_GetAxisDelta(void)
{
    MotorId motor;
    int32_t currentStep;

    /*
     * 这里读的是 STEP 调度器的软件计数，只能证明 MCU 发过多少个脉冲。
     * 没有驱动器反馈/限位/编码器时，不能据此证明云台真实转到 90 度。
     */
    if ((g_gimbalMotorTest.stage != GIMBAL_MOTOR_TEST_STAGE_LEFT_RIGHT_90) &&
        (g_gimbalMotorTest.stage != GIMBAL_MOTOR_TEST_STAGE_UP_DOWN_90)) {
        return 0U;
    }

    motor = GimbalMotorTest_GetStageMotor(g_gimbalMotorTest.stage);
    currentStep = Motor_GetStepCount(motor);
    return GimbalMotorTest_Abs32(currentStep -
        g_gimbalMotorTest.axisBaseStep);
}

static void GimbalMotorTest_AdvanceStage(void)
{
    if (g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_ZERO) {
        GimbalMotorTest_EnterStage(GIMBAL_MOTOR_TEST_STAGE_LEFT_RIGHT_90);
        return;
    }

    if (g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_LEFT_RIGHT_90) {
        if (CAR_GIMBAL_MOTOR_TEST_RUN_UP_DOWN != 0U) {
            GimbalMotorTest_EnterStage(GIMBAL_MOTOR_TEST_STAGE_UP_DOWN_90);
            return;
        }
        if (CAR_GIMBAL_MOTOR_TEST_RUN_SIM_TRACK != 0U) {
            GimbalMotorTest_EnterStage(GIMBAL_MOTOR_TEST_STAGE_SIM_TRACK);
            return;
        }
        if (CAR_GIMBAL_MOTOR_TEST_RUN_CIRCLE != 0U) {
            GimbalMotorTest_EnterStage(GIMBAL_MOTOR_TEST_STAGE_CIRCLE);
            return;
        }
    }

    if (g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_UP_DOWN_90) {
        if (CAR_GIMBAL_MOTOR_TEST_RUN_SIM_TRACK != 0U) {
            GimbalMotorTest_EnterStage(GIMBAL_MOTOR_TEST_STAGE_SIM_TRACK);
            return;
        }
        if (CAR_GIMBAL_MOTOR_TEST_RUN_CIRCLE != 0U) {
            GimbalMotorTest_EnterStage(GIMBAL_MOTOR_TEST_STAGE_CIRCLE);
            return;
        }
    }

    if ((g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_SIM_TRACK) &&
        (CAR_GIMBAL_MOTOR_TEST_RUN_CIRCLE != 0U)) {
        GimbalMotorTest_EnterStage(GIMBAL_MOTOR_TEST_STAGE_CIRCLE);
        return;
    }

    GimbalMotorTest_EnterStage(GIMBAL_MOTOR_TEST_STAGE_DONE);
}

void GimbalMotorTest_Init(void)
{
    g_gimbalMotorTest.stage = GIMBAL_MOTOR_TEST_STAGE_IDLE;
    g_gimbalMotorTest.axisBaseStep = 0;
    g_gimbalMotorTest.zeroTicks = 0U;
    g_gimbalMotorTest.stageTicks = 0U;
    g_gimbalMotorTest.running = 0U;
}

void GimbalMotorTest_Start(void)
{
    g_gimbalMotorTest.running = 1U;
    GimbalMotorTest_EnterStage(GIMBAL_MOTOR_TEST_STAGE_ZERO);
}

void GimbalMotorTest_Stop(void)
{
    GimbalMotorTest_StopAxes();
    g_gimbalMotorTest.stage = GIMBAL_MOTOR_TEST_STAGE_IDLE;
    g_gimbalMotorTest.axisBaseStep = 0;
    g_gimbalMotorTest.zeroTicks = 0U;
    g_gimbalMotorTest.stageTicks = 0U;
    g_gimbalMotorTest.running = 0U;
}

void GimbalMotorTest_Task(void)
{
    if (!g_gimbalMotorTest.running) {
        return;
    }

    if (g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_ZERO) {
        ++g_gimbalMotorTest.zeroTicks;
        if (g_gimbalMotorTest.zeroTicks >= CAR_GIMBAL_MOTOR_TEST_ZERO_TICKS) {
            GimbalMotorTest_AdvanceStage();
        }
        return;
    }

    if (g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_SIM_TRACK) {
        uint32_t scriptIndex;

        ++g_gimbalMotorTest.stageTicks;
        if (g_gimbalMotorTest.stageTicks >=
            GimbalMotorTest_GetSimTotalTicks()) {
            GimbalMotorTest_AdvanceStage();
            return;
        }

        scriptIndex = (uint32_t)g_gimbalMotorTest.stageTicks /
            (uint32_t)CAR_GIMBAL_MOTOR_TEST_SIM_HOLD_TICKS;
        GimbalMotorTest_ApplySimVision(scriptIndex);
        return;
    }

    if (g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_CIRCLE) {
        uint32_t phase;

        ++g_gimbalMotorTest.stageTicks;
        if (g_gimbalMotorTest.stageTicks >=
            GimbalMotorTest_GetCircleTotalTicks()) {
            GimbalMotorTest_AdvanceStage();
            return;
        }

        phase = ((uint32_t)g_gimbalMotorTest.stageTicks /
            (uint32_t)CAR_GIMBAL_MOTOR_TEST_CIRCLE_PHASE_TICKS) %
            (uint32_t)GIMBAL_MOTOR_TEST_CIRCLE_PHASE_COUNT;
        GimbalMotorTest_ApplyCircle(phase);
        return;
    }

    if (GimbalMotorTest_GetAxisDelta() >=
        GimbalMotorTest_GetStageTargetSteps()) {
        GimbalMotorTest_AdvanceStage();
    }
}

uint8_t GimbalMotorTest_IsRunning(void)
{
    return g_gimbalMotorTest.running;
}

GimbalMotorTestStage GimbalMotorTest_GetStage(void)
{
    return g_gimbalMotorTest.stage;
}

const char *GimbalMotorTest_GetStageName(void)
{
    switch (g_gimbalMotorTest.stage) {
    case GIMBAL_MOTOR_TEST_STAGE_ZERO:
        return "Zero";
    case GIMBAL_MOTOR_TEST_STAGE_LEFT_RIGHT_90:
        return "LR 90";
    case GIMBAL_MOTOR_TEST_STAGE_UP_DOWN_90:
        return "UD 90";
    case GIMBAL_MOTOR_TEST_STAGE_SIM_TRACK:
        return "SimTrack";
    case GIMBAL_MOTOR_TEST_STAGE_CIRCLE:
        return "Circle";
    case GIMBAL_MOTOR_TEST_STAGE_DONE:
        return "Done";
    case GIMBAL_MOTOR_TEST_STAGE_IDLE:
    default:
        return "Idle";
    }
}

uint8_t GimbalMotorTest_GetProgressPercent(void)
{
    uint32_t delta = GimbalMotorTest_GetAxisDelta();
    uint32_t percent;

    if (g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_DONE) {
        return 100U;
    }
    if (g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_ZERO) {
        percent = ((uint32_t)g_gimbalMotorTest.zeroTicks * 100U) /
            CAR_GIMBAL_MOTOR_TEST_ZERO_TICKS;
        return (percent > 100U) ? 100U : (uint8_t)percent;
    }
    if ((g_gimbalMotorTest.stage != GIMBAL_MOTOR_TEST_STAGE_LEFT_RIGHT_90) &&
        (g_gimbalMotorTest.stage != GIMBAL_MOTOR_TEST_STAGE_UP_DOWN_90)) {
        if (g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_SIM_TRACK) {
            percent = ((uint32_t)g_gimbalMotorTest.stageTicks * 100U) /
                GimbalMotorTest_GetSimTotalTicks();
            return (percent > 100U) ? 100U : (uint8_t)percent;
        }
        if (g_gimbalMotorTest.stage == GIMBAL_MOTOR_TEST_STAGE_CIRCLE) {
            percent = ((uint32_t)g_gimbalMotorTest.stageTicks * 100U) /
                GimbalMotorTest_GetCircleTotalTicks();
            return (percent > 100U) ? 100U : (uint8_t)percent;
        }
        return 0U;
    }

    percent = (delta * 100U) / GimbalMotorTest_GetStageTargetSteps();
    return (percent > 100U) ? 100U : (uint8_t)percent;
}
