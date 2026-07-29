/*
 * 二维视觉云台的实车调参表：只保存point与circle两套PD/前馈和限幅参数。
 * 比赛时可直接修改下方常量结构体；运行中通过只读指针提供给gimbal.c。
 * 距离增益不在这里分档，vision.c会根据每帧目标长度连续计算yaw缩放。
 */
#include "staticconfig.h"

#include "library_config.h"

#if CAR_PROFILE_IS_FULL

/*
 * 云台视觉只保留两套基准参数：point使用第1/2个误差，circle使用第3/4个。
 * 距离不再复制参数表；vision.c根据第5字段目标长度连续拟合yaw增益。
 * 这里的maxSpeedX=500是旧中档基准，长度拟合后远端0.8倍约400 SPS、
 * 近端1.6倍约800 SPS，与删除前的三档yaw上限保持一致。
 */

/* 默认增益缩放：反馈和视觉速度前馈都除以100。 */
#define STATICCONFIG_GAIN_SCALE_DEFAULT (100U)

#if CAR_LIBRARY_GIMBAL_TRACKING_ENABLED

/* 矩形中心点基准参数；只用于centerDx/centerDy，不保存距离档。 */
static const StaticConfigGimbalTask g_taskPoint = {
    .name = "point",
    .mode = STATICCONFIG_MODE_CENTER,
    .deadbandX = 0U,
    .deadbandY = 0U,
    .restartDeadbandX = 12U,
    .restartDeadbandY = 12U,
    .kpX = 250U,
    .kpY = 100U,
    .kdX = 40U,
    .kdY = 20U,
    .kffX = 75U,
    .kffY = 25U,
    .gainScale = STATICCONFIG_GAIN_SCALE_DEFAULT,
    .minSpeedX = 0U,
    .minSpeedY = 0U,
    .maxSpeedX = 500U,
    .maxSpeedY = 400U,
    .offsetX = 0,
    .offsetY = 0,
    .useCircleError = 0U
};

/* 圆点基准参数；只用于circleDx/circleDy，不保存距离档。 */
static const StaticConfigGimbalTask g_taskCircle = {
    .name = "circle",
    .mode = STATICCONFIG_MODE_CIRCLE,
    .deadbandX = 0U,
    .deadbandY = 0U,
    .restartDeadbandX = 12U,
    .restartDeadbandY = 12U,
    .kpX = 200U,
    .kpY = 20U,
    .kdX = 0U,
    .kdY = 0U,
    .kffX = 50U,
    .kffY = 5U,
    .gainScale = STATICCONFIG_GAIN_SCALE_DEFAULT,
    .minSpeedX = 0U,
    .minSpeedY = 0U,
    .maxSpeedX = 500U,
    .maxSpeedY = 400U,
    .offsetX = 0,
    .offsetY = 0,
    .useCircleError = 1U
};

#else

/* 二维视觉库关闭时的只读占位表，状态机不会进入视觉比赛任务。 */
static const StaticConfigGimbalTask g_taskPoint = {
    .name = "disabled",
    .mode = STATICCONFIG_MODE_CENTER,
    .gainScale = STATICCONFIG_GAIN_SCALE_DEFAULT
};

static const StaticConfigGimbalTask g_taskCircle = {
    .name = "disabled",
    .mode = STATICCONFIG_MODE_CIRCLE,
    .gainScale = STATICCONFIG_GAIN_SCALE_DEFAULT,
    .useCircleError = 1U
};

#endif

static const StaticConfigGimbalTask *const g_tasks[STATICCONFIG_TASK_COUNT] = {
    &g_taskPoint,
    &g_taskCircle
};

static StaticConfigTaskId g_activeTask = STATICCONFIG_TASK_POINT;

/* 仅供本模块校验两套参数编号；不要用它校验旧距离枚举。 */
static uint8_t StaticConfig_IsValidTask(StaticConfigTaskId taskId)
{
    return ((uint8_t)taskId < (uint8_t)STATICCONFIG_TASK_COUNT) ? 1U : 0U;
}

/* 初始化默认点模式；不读取视觉长度，也不产生电机输出。 */
void StaticConfig_Init(void)
{
    g_activeTask = STATICCONFIG_TASK_POINT;
}

/* 返回当前点/圆参数编号；调用方不得据此推断距离。 */
StaticConfigTaskId StaticConfig_GetActiveTask(void)
{
    return g_activeTask;
}

/* 切换点/圆参数；非法值保持当前配置，函数本身不启动云台。 */
void StaticConfig_SetActiveTask(StaticConfigTaskId taskId)
{
    if (StaticConfig_IsValidTask(taskId) != 0U) {
        g_activeTask = taskId;
    }
}

/* 按编号读取只读参数；非法值回退到point，禁止修改返回对象。 */
const StaticConfigGimbalTask *StaticConfig_GetTask(StaticConfigTaskId taskId)
{
    if (StaticConfig_IsValidTask(taskId) == 0U) {
        taskId = STATICCONFIG_TASK_POINT;
    }
    return g_tasks[(uint8_t)taskId];
}

/* 给视觉控制器读取当前只读参数；不包含动态距离增益。 */
const StaticConfigGimbalTask *StaticConfig_GetActiveGimbal(void)
{
    return StaticConfig_GetTask(g_activeTask);
}

/* 只按视觉误差来源切换参数；距离增益由每帧stageScale单独计算。 */
void StaticConfig_SetActiveMode(StaticConfigMode mode)
{
    StaticConfig_SetActiveTask((mode == STATICCONFIG_MODE_CIRCLE) ?
        STATICCONFIG_TASK_CIRCLE : STATICCONFIG_TASK_POINT);
}

#endif
