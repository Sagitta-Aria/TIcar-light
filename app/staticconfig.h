#ifndef STATICCONFIG_H
#define STATICCONFIG_H

#include <stdint.h>

/*
 * staticconfig：云台视觉闭环的集中调参入口。
 *
 * 视觉脚本当前发送 "centerDx,centerDy;circleDx,circleDy\n"。
 * dx/dy 的符号已经是 target - current，进入 MCU 后统一放大为 0.1 像素单位。
 *
 * 调参时优先改 app/staticconfig.c 里的六个 g_task* 结构体：
 * 近/中/远三个距离档，每个距离档再分中心误差 CENTER 和圆点误差 CIRCLE。
 * board_config.h 只保留方向、超时、pitch 限幅、测试速度等硬件/测试参数。
 */

/* 距离档：后续由小车旋转角度或路线状态判断当前位置。 */
typedef enum {
    STATICCONFIG_DISTANCE_NEAR = 0,
    STATICCONFIG_DISTANCE_MID,
    STATICCONFIG_DISTANCE_FAR,
    STATICCONFIG_DISTANCE_COUNT
} StaticConfigDistance;

/* 视觉模式：CENTER 用前两个误差，CIRCLE 用后两个误差。 */
typedef enum {
    STATICCONFIG_MODE_CENTER = 0,
    STATICCONFIG_MODE_CIRCLE,
    STATICCONFIG_MODE_COUNT
} StaticConfigMode;

/* 六套任务参数编号：三个距离档 x 两种视觉模式。 */
typedef enum {
    STATICCONFIG_TASK_NEAR_CENTER = 0,
    STATICCONFIG_TASK_NEAR_CIRCLE,
    STATICCONFIG_TASK_MID_CENTER,
    STATICCONFIG_TASK_MID_CIRCLE,
    STATICCONFIG_TASK_FAR_CENTER,
    STATICCONFIG_TASK_FAR_CIRCLE,
    STATICCONFIG_TASK_COUNT
} StaticConfigTaskId;

/*
 * StaticConfigGimbalTask：一套云台视觉闭环参数。
 *
 * 使用场景：
 * - 直接改 g_taskNearCenter/g_taskNearCircle/... 的字段来调实车。
 * - App 或路线层根据当前位置调用 StaticConfig_SetActiveTask() 切换参数。
 * - 如果按旋转角自动分段，调用 StaticConfig_UpdateByTurnAngleDeg()。
 *
 * 控制公式：
 * commandSps = (error0.1px * kp + deltaError0.1px * kd) / gainScale
 * 超过死区后若算出来太小，会抬到 minSpeed；超过 maxSpeed 会限幅。
 */
typedef struct {
    /* name：调试名，不参与控制。 */
    const char *name;
    /* distance/mode：这套参数所属距离档和视觉模式。 */
    StaticConfigDistance distance;
    StaticConfigMode mode;
    /* turnAngleMin/MaxDeg：累计旋转角分段范围，闭区间，单位度。 */
    int16_t turnAngleMinDeg;
    int16_t turnAngleMaxDeg;
    /* deadbandX/Y：停止阈值，单位 0.1 像素；10 表示 1 像素。 */
    uint16_t deadbandX;
    uint16_t deadbandY;
    /* restartDeadbandX/Y：停止后重新动作的阈值，必须不小于停止阈值。 */
    uint16_t restartDeadbandX;
    uint16_t restartDeadbandY;
    /* kpX/Y：比例增益；配合 gainScale 换算成云台 SPS。 */
    uint16_t kpX;
    uint16_t kpY;
    /* kdX/Y：相邻视觉帧误差变化的阻尼增益；不用 D 时填 0。 */
    uint16_t kdX;
    uint16_t kdY;
    /* gainScale：P/D 统一除数，默认 100，避免浮点运算。 */
    uint16_t gainScale;
    /* minSpeedX/Y：超过死区后的最小动作速度，单位 SPS。 */
    uint16_t minSpeedX;
    uint16_t minSpeedY;
    /* maxSpeedX/Y：单套参数自己的速度上限，单位 SPS。 */
    uint16_t maxSpeedX;
    uint16_t maxSpeedY;
    /* offsetX/Y：加到 target-current 误差上的安装补偿，单位 0.1 像素。 */
    int16_t offsetX;
    int16_t offsetY;
    /* useCircleError：0 用 centerDx/centerDy，1 用 circleDx/circleDy。 */
    uint8_t useCircleError;
} StaticConfigGimbalTask;

/* StaticConfig_Init：初始化当前任务配置；默认任务在 staticconfig.c 里设置。 */
void StaticConfig_Init(void);

/* StaticConfig_GetActiveTask：读取当前 active 任务编号。 */
StaticConfigTaskId StaticConfig_GetActiveTask(void);

/* StaticConfig_SetActiveTask：按任务编号切换 active 参数，非法编号会保持不变。 */
void StaticConfig_SetActiveTask(StaticConfigTaskId taskId);

/* StaticConfig_GetTask：按任务编号读取参数，非法编号返回默认参数。 */
const StaticConfigGimbalTask *StaticConfig_GetTask(StaticConfigTaskId taskId);

/* StaticConfig_GetActiveGimbal：读取当前云台闭环使用的参数。 */
const StaticConfigGimbalTask *StaticConfig_GetActiveGimbal(void);

/* StaticConfig_GetTaskFor：按距离档和模式返回对应任务编号。 */
StaticConfigTaskId StaticConfig_GetTaskFor(StaticConfigDistance distance,
    StaticConfigMode mode);

/* StaticConfig_SetActiveByDistanceMode：按距离档和模式直接切换 active 参数。 */
void StaticConfig_SetActiveByDistanceMode(StaticConfigDistance distance,
    StaticConfigMode mode);

/*
 * StaticConfig_UpdateByTurnAngleDeg：用累计旋转角度和模式自动选择 active 参数。
 * 说明：角度范围来自 staticconfig.c 里的六个任务结构体。
 * 注意：只有调用本函数后 active 才会随角度变化；不调用时保持 Init/Set 选择的任务。
 */
void StaticConfig_UpdateByTurnAngleDeg(int16_t turnAngleDeg,
    StaticConfigMode mode);

#endif
