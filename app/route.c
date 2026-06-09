#include "route.h"

#include "board_config.h"
#include "jy61p.h"
#include "log_uart.h"
#include "motor.h"

/* RouteProfile：路线外环当前速度配置，单位统一为 SPS。 */
typedef struct {
    uint16_t targetBaseSps;
    uint16_t currentBaseSps;
    uint16_t targetTurnLimit;
    uint16_t currentTurnLimit;
} RouteProfile;

static RouteStage g_routeStage;
static RouteProfile g_routeProfile;
static uint8_t g_routeRunning;
static uint8_t g_routeCornerIndex;
static uint16_t g_routeStageTicks;
static int32_t g_routeLeftStepBase;
static int32_t g_routeRightStepBase;
static int16_t g_routeCornerStartYaw;
static int16_t g_routeTargetYaw;

/*
 * 作用：把路线阶段枚举转成日志字符串。
 * 使用场景：阶段切换日志和外部调试显示。
 */
static const char *Route_StageText(RouteStage stage)
{
    switch (stage) {
    case ROUTE_STAGE_IDLE:
        return "idle";
    case ROUTE_STAGE_STRAIGHT:
        return "straight";
    case ROUTE_STAGE_APPROACH_CORNER:
        return "approach";
    case ROUTE_STAGE_TURNING:
        return "turning";
    case ROUTE_STAGE_EXIT_CORNER:
        return "exit";
    default:
        return "unknown";
    }
}

/*
 * 作用：把 int32_t 转成绝对值，避免反向 STEP 计数影响距离判断。
 * 使用场景：路线层计算相对 STEP 位移。
 */
static uint32_t Route_Abs32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

/*
 * 作用：把角度归一化到 -180~180，便于比较转了多少度。
 * 使用场景：JY61P yaw 和目标角度的差值计算。
 */
static int16_t Route_NormalizeAngle(int32_t angle)
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
 * 作用：计算两个角度之间的最短差值。
 * 使用场景：判断直角是否已经转够。
 */
static int16_t Route_AngleDiff(int16_t current, int16_t target)
{
    return Route_NormalizeAngle((int32_t)current - (int32_t)target);
}

/*
 * 作用：把当前值向目标值平滑靠近，避免速度突跳。
 * 使用场景：路线层在直道、入弯、出弯之间切换时。
 */
static uint16_t Route_StepToward(uint16_t current, uint16_t target)
{
    uint16_t delta;

    if (current < target) {
        delta = (uint16_t)(target - current);
        if (delta > CAR_ROUTE_PROFILE_STEP) {
            delta = CAR_ROUTE_PROFILE_STEP;
        }
        return (uint16_t)(current + delta);
    }

    if (current > target) {
        delta = (uint16_t)(current - target);
        if (delta > CAR_ROUTE_PROFILE_STEP) {
            delta = CAR_ROUTE_PROFILE_STEP;
        }
        return (uint16_t)(current - delta);
    }

    return current;
}

/*
 * 作用：把当前路线阶段对应的目标速度写进去。
 * 使用场景：路线阶段切换时。
 */
static void Route_SetTargets(RouteStage stage)
{
    switch (stage) {
    case ROUTE_STAGE_STRAIGHT:
        g_routeProfile.targetBaseSps = CAR_ROUTE_CRUISE_SPEED_SPS;
        g_routeProfile.targetTurnLimit = CAR_ROUTE_CRUISE_TURN_LIMIT;
        break;

    case ROUTE_STAGE_APPROACH_CORNER:
        g_routeProfile.targetBaseSps = CAR_ROUTE_APPROACH_SPEED_SPS;
        g_routeProfile.targetTurnLimit = CAR_ROUTE_APPROACH_TURN_LIMIT;
        break;

    case ROUTE_STAGE_TURNING:
        g_routeProfile.targetBaseSps = CAR_ROUTE_TURN_SPEED_SPS;
        g_routeProfile.targetTurnLimit = CAR_ROUTE_TURN_LIMIT;
        break;

    case ROUTE_STAGE_EXIT_CORNER:
        g_routeProfile.targetBaseSps = CAR_ROUTE_EXIT_SPEED_SPS;
        g_routeProfile.targetTurnLimit = CAR_ROUTE_APPROACH_TURN_LIMIT;
        break;

    case ROUTE_STAGE_IDLE:
    default:
        g_routeProfile.targetBaseSps = CAR_TRACK_BASE_SPEED_SPS;
        g_routeProfile.targetTurnLimit = CAR_ROUTE_CRUISE_TURN_LIMIT;
        break;
    }
}

/*
 * 作用：进入某个路线阶段，并重置该阶段的计数器。
 * 使用场景：路线外环从直道进入拐角、从拐角回到直道。
 */
static void Route_EnterStage(RouteStage stage)
{
    g_routeStage = stage;
    g_routeStageTicks = 0U;
    Route_SetTargets(stage);
    LOG_RAW("route: stage=");
    LOG_LINE(Route_StageText(stage));

    if (stage == ROUTE_STAGE_TURNING) {
        g_routeCornerStartYaw = JY61P_GetYawDeg();
        if (CAR_ROUTE_TURN_IS_LEFT) {
            g_routeTargetYaw = (int16_t)(g_routeCornerStartYaw + 90);
        } else {
            g_routeTargetYaw = (int16_t)(g_routeCornerStartYaw - 90);
        }
        g_routeTargetYaw = Route_NormalizeAngle(g_routeTargetYaw);
    } else if (stage == ROUTE_STAGE_STRAIGHT) {
        g_routeLeftStepBase = Motor_GetStepCount(MOTOR_CHASSIS_LEFT);
        g_routeRightStepBase = Motor_GetStepCount(MOTOR_CHASSIS_RIGHT);
    }
}

/*
 * 作用：计算当前边已经输出了多少 STEP。
 * 使用场景：判断是否接近直角和是否已经到达直角。
 * 说明：这里统计的是 MCU 实际发给闭环步进驱动器的脉冲数。
 */
static uint32_t Route_GetTravelSteps(void)
{
    int32_t leftDelta =
        Motor_GetStepCount(MOTOR_CHASSIS_LEFT) - g_routeLeftStepBase;
    int32_t rightDelta =
        Motor_GetStepCount(MOTOR_CHASSIS_RIGHT) - g_routeRightStepBase;
    uint32_t leftSteps = Route_Abs32(leftDelta);
    uint32_t rightSteps = Route_Abs32(rightDelta);

    return (leftSteps + rightSteps) / 2U;
}

/*
 * 作用：初始化路线外环状态。
 * 使用场景：App_Init 或 Route_Start 前调用。
 * 说明：只复位路线内部计数和速度配置，不直接输出电机命令。
 */
void Route_Init(void)
{
    g_routeRunning = 0U;
    g_routeCornerIndex = 0U;
    g_routeStageTicks = 0U;
    g_routeLeftStepBase = Motor_GetStepCount(MOTOR_CHASSIS_LEFT);
    g_routeRightStepBase = Motor_GetStepCount(MOTOR_CHASSIS_RIGHT);
    g_routeCornerStartYaw = 0;
    g_routeTargetYaw = 0;
    g_routeProfile.currentBaseSps = CAR_TRACK_BASE_SPEED_SPS;
    g_routeProfile.currentTurnLimit = CAR_ROUTE_CRUISE_TURN_LIMIT;
    g_routeProfile.targetBaseSps = CAR_TRACK_BASE_SPEED_SPS;
    g_routeProfile.targetTurnLimit = CAR_ROUTE_CRUISE_TURN_LIMIT;
    g_routeStage = ROUTE_STAGE_IDLE;
}

/*
 * 作用：启动路线外环。
 * 使用场景：状态机进入正式循迹任务时。
 * 说明：会重新取当前底盘 STEP 作为直道起点，并进入直道阶段。
 */
void Route_Start(void)
{
    Route_Init();
    g_routeRunning = 1U;
    LOG_LINE("route: start");
    Route_EnterStage(ROUTE_STAGE_STRAIGHT);
}

/*
 * 作用：停止路线外环并恢复默认速度配置。
 * 使用场景：返回菜单、停止、错误或切换到单独测试页时。
 */
void Route_Stop(void)
{
    if (g_routeRunning != 0U) {
        LOG_LINE("route: stop");
    }
    g_routeRunning = 0U;
    g_routeStage = ROUTE_STAGE_IDLE;
    g_routeStageTicks = 0U;
    g_routeProfile.currentBaseSps = CAR_TRACK_BASE_SPEED_SPS;
    g_routeProfile.currentTurnLimit = CAR_ROUTE_CRUISE_TURN_LIMIT;
    g_routeProfile.targetBaseSps = CAR_TRACK_BASE_SPEED_SPS;
    g_routeProfile.targetTurnLimit = CAR_ROUTE_CRUISE_TURN_LIMIT;
}

/*
 * 作用：周期更新路线阶段和速度目标。
 * 使用场景：正式循迹状态下每轮主循环调用。
 * 说明：本模块只给 Tracking 提供基础速度和转向限制，不直接控制电机。
 */
void Route_Task(void)
{
    uint32_t travelSteps;
    int16_t yawError;

    if (!g_routeRunning) {
        return;
    }

    ++g_routeStageTicks;
    travelSteps = Route_GetTravelSteps();

    switch (g_routeStage) {
    case ROUTE_STAGE_STRAIGHT:
        if (travelSteps >= CAR_ROUTE_APPROACH_STEPS) {
            Route_EnterStage(ROUTE_STAGE_APPROACH_CORNER);
        }
        break;

    case ROUTE_STAGE_APPROACH_CORNER:
        if (travelSteps >= CAR_ROUTE_EDGE_STEPS) {
            Route_EnterStage(ROUTE_STAGE_TURNING);
        }
        break;

    case ROUTE_STAGE_TURNING:
        if (JY61P_HasYaw()) {
            yawError = Route_AngleDiff(JY61P_GetYawDeg(), g_routeTargetYaw);
            if ((yawError <= CAR_ROUTE_TURN_TOLERANCE_DEG) &&
                (yawError >= (int16_t)(-CAR_ROUTE_TURN_TOLERANCE_DEG))) {
                Route_EnterStage(ROUTE_STAGE_EXIT_CORNER);
            }
        } else if (g_routeStageTicks >= CAR_ROUTE_TURN_HOLD_TICKS) {
            Route_EnterStage(ROUTE_STAGE_EXIT_CORNER);
        }
        break;

    case ROUTE_STAGE_EXIT_CORNER:
        if (g_routeStageTicks >= CAR_ROUTE_EXIT_TICKS) {
            g_routeCornerIndex = (uint8_t)((g_routeCornerIndex + 1U) %
                CAR_ROUTE_CORNER_COUNT);
            Route_EnterStage(ROUTE_STAGE_STRAIGHT);
        }
        break;

    case ROUTE_STAGE_IDLE:
    default:
        break;
    }

    g_routeProfile.currentBaseSps = Route_StepToward(
        g_routeProfile.currentBaseSps, g_routeProfile.targetBaseSps);
    g_routeProfile.currentTurnLimit = Route_StepToward(
        g_routeProfile.currentTurnLimit, g_routeProfile.targetTurnLimit);
}

/* 作用：返回路线外环是否正在运行。 */
uint8_t Route_IsRunning(void)
{
    return g_routeRunning;
}

/* 作用：返回当前路线阶段。 */
RouteStage Route_GetStage(void)
{
    return g_routeStage;
}

/* 作用：返回路线阶段字符串，用于日志/OLED 调试。 */
const char *Route_GetStageName(RouteStage stage)
{
    return Route_StageText(stage);
}

/* 作用：返回当前平滑后的基础底盘速度，单位 SPS。 */
uint16_t Route_GetBaseSpeedSps(void)
{
    return g_routeProfile.currentBaseSps;
}

/* 作用：返回当前平滑后的最大转向修正限制。 */
uint16_t Route_GetTurnLimit(void)
{
    return g_routeProfile.currentTurnLimit;
}

/* 作用：返回已经通过的拐角编号，后续任务流程可据此分段。 */
uint8_t Route_GetCornerIndex(void)
{
    return g_routeCornerIndex;
}
