#include "route.h"

#include "board_config.h"
#include "encoder.h"
#include "jy61p.h"

/* RouteProfile：路线外环当前速度配置。 */
typedef struct {
    uint16_t targetBaseDuty;
    uint16_t currentBaseDuty;
    uint16_t targetTurnLimit;
    uint16_t currentTurnLimit;
} RouteProfile;

static RouteStage g_routeStage;
static RouteProfile g_routeProfile;
static uint8_t g_routeRunning;
static uint8_t g_routeCornerIndex;
static uint16_t g_routeStageTicks;
static int32_t g_routeLeftBase;
static int32_t g_routeRightBase;
static int16_t g_routeCornerStartYaw;
static int16_t g_routeTargetYaw;

/*
 * 作用：把 int32_t 转成绝对值，避免编码器反向计数影响距离判断。
 * 使用场景：路线层计算相对编码器位移。
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
        g_routeProfile.targetBaseDuty = CAR_ROUTE_CRUISE_DUTY;
        g_routeProfile.targetTurnLimit = CAR_ROUTE_CRUISE_TURN_LIMIT;
        break;

    case ROUTE_STAGE_APPROACH_CORNER:
        g_routeProfile.targetBaseDuty = CAR_ROUTE_APPROACH_DUTY;
        g_routeProfile.targetTurnLimit = CAR_ROUTE_APPROACH_TURN_LIMIT;
        break;

    case ROUTE_STAGE_TURNING:
        g_routeProfile.targetBaseDuty = CAR_ROUTE_TURN_DUTY;
        g_routeProfile.targetTurnLimit = CAR_ROUTE_TURN_LIMIT;
        break;

    case ROUTE_STAGE_EXIT_CORNER:
        g_routeProfile.targetBaseDuty = CAR_ROUTE_EXIT_DUTY;
        g_routeProfile.targetTurnLimit = CAR_ROUTE_APPROACH_TURN_LIMIT;
        break;

    case ROUTE_STAGE_IDLE:
    default:
        g_routeProfile.targetBaseDuty = CAR_TRACK_BASE_DUTY;
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

    if (stage == ROUTE_STAGE_TURNING) {
        g_routeCornerStartYaw = JY61P_GetYawDeg();
        if (CAR_ROUTE_TURN_IS_LEFT) {
            g_routeTargetYaw = (int16_t)(g_routeCornerStartYaw + 90);
        } else {
            g_routeTargetYaw = (int16_t)(g_routeCornerStartYaw - 90);
        }
        g_routeTargetYaw = Route_NormalizeAngle(g_routeTargetYaw);
    } else if (stage == ROUTE_STAGE_STRAIGHT) {
        g_routeLeftBase = Encoder_GetLeft();
        g_routeRightBase = Encoder_GetRight();
    }
}

/*
 * 作用：计算当前边已经走了多少编码器相对计数。
 * 使用场景：判断是否接近直角和是否已经到达直角。
 */
static uint32_t Route_GetTravelTicks(void)
{
    int32_t leftDelta = Encoder_GetLeft() - g_routeLeftBase;
    int32_t rightDelta = Encoder_GetRight() - g_routeRightBase;
    uint32_t leftTicks = Route_Abs32(leftDelta);
    uint32_t rightTicks = Route_Abs32(rightDelta);

    return (leftTicks + rightTicks) / 2U;
}

void Route_Init(void)
{
    g_routeRunning = 0U;
    g_routeCornerIndex = 0U;
    g_routeStageTicks = 0U;
    g_routeLeftBase = Encoder_GetLeft();
    g_routeRightBase = Encoder_GetRight();
    g_routeCornerStartYaw = 0;
    g_routeTargetYaw = 0;
    g_routeProfile.currentBaseDuty = CAR_TRACK_BASE_DUTY;
    g_routeProfile.currentTurnLimit = CAR_ROUTE_CRUISE_TURN_LIMIT;
    g_routeProfile.targetBaseDuty = CAR_TRACK_BASE_DUTY;
    g_routeProfile.targetTurnLimit = CAR_ROUTE_CRUISE_TURN_LIMIT;
    g_routeStage = ROUTE_STAGE_IDLE;
}

void Route_Start(void)
{
    Route_Init();
    g_routeRunning = 1U;
    Route_EnterStage(ROUTE_STAGE_STRAIGHT);
}

void Route_Stop(void)
{
    g_routeRunning = 0U;
    g_routeStage = ROUTE_STAGE_IDLE;
    g_routeStageTicks = 0U;
    g_routeProfile.currentBaseDuty = CAR_TRACK_BASE_DUTY;
    g_routeProfile.currentTurnLimit = CAR_ROUTE_CRUISE_TURN_LIMIT;
    g_routeProfile.targetBaseDuty = CAR_TRACK_BASE_DUTY;
    g_routeProfile.targetTurnLimit = CAR_ROUTE_CRUISE_TURN_LIMIT;
}

void Route_Task(void)
{
    uint32_t travelTicks;
    int16_t yawError;

    if (!g_routeRunning) {
        return;
    }

    ++g_routeStageTicks;
    travelTicks = Route_GetTravelTicks();

    switch (g_routeStage) {
    case ROUTE_STAGE_STRAIGHT:
        if (travelTicks >= CAR_ROUTE_APPROACH_TICKS) {
            Route_EnterStage(ROUTE_STAGE_APPROACH_CORNER);
        }
        break;

    case ROUTE_STAGE_APPROACH_CORNER:
        if (travelTicks >= CAR_ROUTE_EDGE_TICKS) {
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

    g_routeProfile.currentBaseDuty = Route_StepToward(
        g_routeProfile.currentBaseDuty, g_routeProfile.targetBaseDuty);
    g_routeProfile.currentTurnLimit = Route_StepToward(
        g_routeProfile.currentTurnLimit, g_routeProfile.targetTurnLimit);
}

uint8_t Route_IsRunning(void)
{
    return g_routeRunning;
}

RouteStage Route_GetStage(void)
{
    return g_routeStage;
}

const char *Route_GetStageName(RouteStage stage)
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

uint16_t Route_GetBaseDuty(void)
{
    return g_routeProfile.currentBaseDuty;
}

uint16_t Route_GetTurnLimit(void)
{
    return g_routeProfile.currentTurnLimit;
}

uint8_t Route_GetCornerIndex(void)
{
    return g_routeCornerIndex;
}
