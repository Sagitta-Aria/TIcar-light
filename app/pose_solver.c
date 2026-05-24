#include "pose_solver.h"

#include "board_config.h"
#include "jy61p.h"
#include "motor.h"

#if (CAR_POSE_STEP_TO_MM_DENOMINATOR == 0)
#error "CAR_POSE_STEP_TO_MM_DENOMINATOR must be greater than 0"
#endif

#define POSE_SOLVER_TRIG_Q             (14)
#define POSE_SOLVER_TRIG_SCALE         (1L << POSE_SOLVER_TRIG_Q)

static PoseSolverPose g_pose;
static int32_t g_lastLeftSteps;
static int32_t g_lastRightSteps;
static int16_t g_yawZeroDeg;
static uint8_t g_hasYawZero;

static int32_t PoseSolver_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int16_t PoseSolver_NormalizeAngle(int32_t angle)
{
    while (angle > 180) {
        angle -= 360;
    }
    while (angle < -180) {
        angle += 360;
    }
    return (int16_t)angle;
}

/*
 * 作用：把角度转成 Q14 正弦值。
 * 说明：使用整数近似，避免主循环引入浮点 sin/cos。
 */
static int16_t PoseSolver_SinDegQ14(int16_t deg)
{
    int32_t angle = deg;
    int32_t x;
    int32_t product;
    int32_t denominator;
    int32_t value;
    int8_t sign = 1;

    while (angle < 0) {
        angle += 360;
    }
    angle %= 360;

    if (angle > 180) {
        angle -= 180;
        sign = -1;
    }

    x = angle;
    product = x * (180 - x);
    if (product == 0) {
        return 0;
    }

    denominator = 40500 - product;
    value = (4L * product * POSE_SOLVER_TRIG_SCALE) / denominator;
    if (value > POSE_SOLVER_TRIG_SCALE) {
        value = POSE_SOLVER_TRIG_SCALE;
    }

    return (int16_t)((sign > 0) ? value : -value);
}

static int16_t PoseSolver_CosDegQ14(int16_t deg)
{
    return PoseSolver_SinDegQ14((int16_t)(deg + 90));
}

/*
 * 作用：把 STEP 增量换算成毫米。
 * 使用场景：底盘里程计积分。
 * 说明：比例参数先放在 board_config.h，实车用尺子标定后再改。
 */
static int32_t PoseSolver_StepsToMm(int32_t steps)
{
    int32_t sign = (steps < 0) ? -1 : 1;
    int32_t absSteps = PoseSolver_Abs32(steps);
    int32_t mm = (absSteps * CAR_POSE_STEP_TO_MM_NUMERATOR +
        (CAR_POSE_STEP_TO_MM_DENOMINATOR / 2)) /
        CAR_POSE_STEP_TO_MM_DENOMINATOR;

    return (sign > 0) ? mm : -mm;
}

static void PoseSolver_UpdateImu(void)
{
    int16_t rollDeg;
    int16_t pitchDeg;
    int16_t yawDeg;

    if (JY61P_GetAnglesDeg(&rollDeg, &pitchDeg, &yawDeg) == 0U) {
        g_pose.hasImu = 0U;
        return;
    }

    if (g_hasYawZero == 0U) {
        g_yawZeroDeg = yawDeg;
        g_hasYawZero = 1U;
    }

    g_pose.rollDeg = rollDeg;
    g_pose.pitchDeg = pitchDeg;
    g_pose.yawDeg = PoseSolver_NormalizeAngle(
        (int32_t)yawDeg - (int32_t)g_yawZeroDeg);
    g_pose.hasImu = 1U;
}

static void PoseSolver_UpdateSteps(void)
{
    int32_t leftSteps = Motor_GetStepCount(MOTOR_CHASSIS_LEFT);
    int32_t rightSteps = Motor_GetStepCount(MOTOR_CHASSIS_RIGHT);
    int32_t deltaLeftSteps = leftSteps - g_lastLeftSteps;
    int32_t deltaRightSteps = rightSteps - g_lastRightSteps;
    int32_t deltaSteps = (deltaLeftSteps + deltaRightSteps) / 2;
    int32_t deltaMm = PoseSolver_StepsToMm(deltaSteps);
    int16_t sinYaw = PoseSolver_SinDegQ14(g_pose.yawDeg);
    int16_t cosYaw = PoseSolver_CosDegQ14(g_pose.yawDeg);

    g_lastLeftSteps = leftSteps;
    g_lastRightSteps = rightSteps;
    g_pose.leftSteps = leftSteps;
    g_pose.rightSteps = rightSteps;
    g_pose.travelMm += deltaMm;
    g_pose.xMm += (deltaMm * (int32_t)sinYaw) / POSE_SOLVER_TRIG_SCALE;
    g_pose.yMm += (deltaMm * (int32_t)cosYaw) / POSE_SOLVER_TRIG_SCALE;
}

void PoseSolver_Init(void)
{
    PoseSolver_Reset();
}

void PoseSolver_Reset(void)
{
    g_pose.xMm = 0;
    g_pose.yMm = 0;
    g_pose.travelMm = 0;
    g_pose.leftSteps = Motor_GetStepCount(MOTOR_CHASSIS_LEFT);
    g_pose.rightSteps = Motor_GetStepCount(MOTOR_CHASSIS_RIGHT);
    g_pose.rollDeg = 0;
    g_pose.pitchDeg = 0;
    g_pose.yawDeg = 0;
    g_pose.hasImu = 0U;
    g_lastLeftSteps = g_pose.leftSteps;
    g_lastRightSteps = g_pose.rightSteps;

    if (JY61P_HasAngles() != 0U) {
        g_yawZeroDeg = JY61P_GetYawDeg();
        g_hasYawZero = 1U;
        PoseSolver_UpdateImu();
    } else {
        g_yawZeroDeg = 0;
        g_hasYawZero = 0U;
    }
}

void PoseSolver_Task(void)
{
    PoseSolver_UpdateImu();
    PoseSolver_UpdateSteps();
}

void PoseSolver_GetPose(PoseSolverPose *pose)
{
    if (pose == 0) {
        return;
    }

    *pose = g_pose;
}

int32_t PoseSolver_GetXmm(void)
{
    return g_pose.xMm;
}

int32_t PoseSolver_GetYmm(void)
{
    return g_pose.yMm;
}

int32_t PoseSolver_GetTravelMm(void)
{
    return g_pose.travelMm;
}

int16_t PoseSolver_GetYawDeg(void)
{
    return g_pose.yawDeg;
}

uint8_t PoseSolver_HasImu(void)
{
    return g_pose.hasImu;
}
