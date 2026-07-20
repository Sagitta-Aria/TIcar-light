#ifndef STATICCONFIG_H
#define STATICCONFIG_H

#include <stdint.h>

/*
 * staticconfig：云台视觉闭环的集中调参入口。
 *
 * 视觉脚本当前发送 "centerDx,centerDy;circleDx,circleDy;stageScale\n"。
 * dx/dy 的符号已经是 target - current，进入 MCU 后统一放大为 0.1 像素单位。
 *
 * 调参时只改 app/staticconfig.c 里的 point/circle 两个结构体。视觉第5字段
 * 提供目标长度，距离造成的yaw响应差异由连续增益拟合，不再切近/中/远表。
 * board_config.h 只保留方向、超时、pitch 限幅、测试速度等硬件/测试参数。
 */

/* Task4强转yaw三段的近/中/远索引；视觉参数选择已不再使用该枚举。 */
typedef enum {
    STATICCONFIG_DISTANCE_NEAR = 0,
    STATICCONFIG_DISTANCE_MID,
    STATICCONFIG_DISTANCE_FAR,
    STATICCONFIG_DISTANCE_COUNT
} StaticConfigDistance;

/* 视觉模式：CENTER 用前两个误差，CIRCLE 用后两个误差；Task4由子菜单选择。 */
typedef enum {
    STATICCONFIG_MODE_CENTER = 0,
    STATICCONFIG_MODE_CIRCLE,
    STATICCONFIG_MODE_COUNT
} StaticConfigMode;

/* 最终只保留矩形中心点和圆点两套视觉参数。 */
typedef enum {
    STATICCONFIG_TASK_POINT = 0,
    STATICCONFIG_TASK_CIRCLE,
    STATICCONFIG_TASK_COUNT
} StaticConfigTaskId;

/*
 * StaticConfigGimbalTask：一套云台视觉闭环参数。
 *
 * 使用场景：
 * - 直接改 g_taskPoint/g_taskCircle 的字段来调实车。
 * - App 只按视觉模式调用 StaticConfig_SetActiveMode() 切换参数。
 *
 * 控制公式：
 * feedbackSps = (error0.1px * kp + deltaError0.1px * kd) / gainScale
 * feedForwardSps = deltaError0.1px * kff / gainScale
 * commandSps = clamp(feedbackSps + feedForwardSps)
 * 反馈超过死区后若算出来太小，会抬到 minSpeed；视觉速度前馈绕过位置死区，
 * 但合成命令仍受 maxSpeed 限幅。
 */
typedef struct {
    /* name：调试名，不参与控制。 */
    const char *name;
    /* mode：这套参数使用矩形中心误差还是圆点误差。 */
    StaticConfigMode mode;
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
    /* kffX/Y：相邻视觉帧误差趋势前馈；不用前馈时填 0。 */
    uint16_t kffX;
    uint16_t kffY;
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

/* StaticConfig_SetActiveMode：只按点/圆模式切换两套最终参数。 */
void StaticConfig_SetActiveMode(StaticConfigMode mode);

#endif
