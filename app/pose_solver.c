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

/*
 * 作用：计算 int32_t 绝对值。
 * 使用场景：STEP 转毫米前先去掉方向符号。
 */
static int32_t PoseSolver_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/*
 * 作用：把角度归一化到 -180~180。
 * 使用场景：yaw 零点扣除后，保持姿态角范围稳定。
 */
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

/*
 * 作用：读取 JY61P 姿态并更新相对 yaw。
 * 使用场景：PoseSolver_Task 每轮调用。
 * 说明：第一次拿到 yaw 时自动作为零点；没有 IMU 数据时只清 hasImu，不阻塞等待。
 */
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

/*
 * 作用：用左右底盘 STEP 增量积分二维位移。
 * 使用场景：PoseSolver_Task 每轮调用。
 * 说明：这里用的是 MCU 已输出 STEP 计数，不等同于带反馈的真实位移。
 */
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

/*
 * 作用：初始化位姿解算模块。
 * 使用场景：App_Init 阶段调用。
 */
void PoseSolver_Init(void)
{
    PoseSolver_Reset();
}

/*
 * 作用：把当前位置作为新的车体位姿零点。
 * 使用场景：上电初始化、任务开始或需要重新计程时。
 */
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

/*
 * 作用：周期更新车体位姿估计。
 * 使用场景：App_Task 每轮调用。
 * 说明：不访问 OLED/串口，不做阻塞等待；可在其它任务前后稳定调用。
 */
void PoseSolver_Task(void)
{
    PoseSolver_UpdateImu();
    PoseSolver_UpdateSteps();
}

/* 作用：复制一份当前位姿快照给调用者。 */
void PoseSolver_GetPose(PoseSolverPose *pose)
{
    if (pose == 0) {
        return;
    }

    *pose = g_pose;
}

/* 作用：返回相对零点的 x 位移，单位毫米。 */
int32_t PoseSolver_GetXmm(void)
{
    return g_pose.xMm;
}

/* 作用：返回相对零点的 y 位移，单位毫米。 */
int32_t PoseSolver_GetYmm(void)
{
    return g_pose.yMm;
}

/* 作用：返回累计里程估计，单位毫米。 */
int32_t PoseSolver_GetTravelMm(void)
{
    return g_pose.travelMm;
}

/* 作用：返回相对零点 yaw，单位度。 */
int16_t PoseSolver_GetYawDeg(void)
{
    return g_pose.yawDeg;
}

/* 作用：返回当前是否有有效 IMU 姿态。 */
uint8_t PoseSolver_HasImu(void)
{
    return g_pose.hasImu;
}
