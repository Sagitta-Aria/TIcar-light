#include "staticconfig.h"

/*
 * 这里是六套云台参数的实际调参表。
 *
 * 调法：
 * 1. 先选距离档：near/mid/far 对应 turnAngleMinDeg ~ turnAngleMaxDeg。
 * 2. 再选视觉模式：center 使用视觉串口的第 1/2 个数，circle 使用第 3/4 个数。
 * 3. 改 deadband、kp、kd、min/maxSpeed、offset 后重新烧录即可生效。
 *
 * 目前 StaticConfig_Init() 默认启用 near_center。
 * 若想上电就跟圆点，把 StaticConfig_Init() 改成 STATICCONFIG_TASK_NEAR_CIRCLE；
 * 若想按旋转角自动切换，需要路线/姿态层周期调用 StaticConfig_UpdateByTurnAngleDeg()。
 */

/* 默认增益缩放：速度 = (误差 * kp + 误差变化 * kd) / 100。 */
#define STATICCONFIG_GAIN_SCALE_DEFAULT (100U)

/*
 * 下面六个结构体就是最终要调的六套参数。
 * turnAngleMinDeg/turnAngleMaxDeg 用来给“旋转角度 -> 当前位置”做分段。
 * 角度范围现在是占位值，实车测出每段角度后直接改这里。
 */
static const StaticConfigGimbalTask g_taskNearCenter = {
    .name = "near_center",                         //调试名：近距离 + 矩形中心
    .distance = STATICCONFIG_DISTANCE_NEAR,         //距离档：近距离
    .mode = STATICCONFIG_MODE_CENTER,               //模式：使用矩形中心误差
    .turnAngleMinDeg = -32768,                      //角度下限：近距离段起点
    .turnAngleMaxDeg = 60,                          //角度上限：近距离段终点
    .deadbandX = 12U,                               //X 死区：1 像素
    .deadbandY = 12U,                               //Y 死区：1 像素
    .restartDeadbandX = 17U,                        //X 重启阈值：停止后超过 1.7 像素再动作
    .restartDeadbandY = 17U,                        //Y 重启阈值：停止后超过 1.7 像素再动作
    .kpX = 200U,                                    //X 比例增益：左右轴响应强度
    .kpY = 100U,                                    //Y 比例增益：上下轴响应强度
    .kdX = 20U,                                      //X 微分增益：不用 D 就填 0
    .kdY = 20U,                                      //Y 微分增益：不用 D 就填 0
    .gainScale = STATICCONFIG_GAIN_SCALE_DEFAULT,   //增益缩放：默认除以 100
    .minSpeedX = 200U,                              //X 最小动作速度：单位 SPS
    .minSpeedY = 100U,                               //Y 最小动作速度：单位 SPS
    .maxSpeedX = 5000U,                            //X 最大速度：单位 SPS
    .maxSpeedY = 50000U,                            //Y 最大速度：单位 SPS
    .offsetX = -160,                                   //X 安装补偿：正值让点向右偏
    .offsetY = 100,                                  //Y 安装补偿：正值让点向左偏
    .useCircleError = 0U                            //误差来源：0 取第 1/2 个数
};

static const StaticConfigGimbalTask g_taskNearCircle = {
    .name = "near_circle",                         //调试名：近距离 + 圆点
    .distance = STATICCONFIG_DISTANCE_NEAR,         //距离档：近距离
    .mode = STATICCONFIG_MODE_CIRCLE,               //模式：使用圆点误差
    .turnAngleMinDeg = -32768,                      //角度下限：近距离段起点
    .turnAngleMaxDeg = 60,                          //角度上限：近距离段终点
    .deadbandX = 10U,                               //X 死区：1 像素
    .deadbandY = 10U,                               //Y 死区：1 像素
    .restartDeadbandX = 15U,                        //X 重启阈值：停止后超过 1.5 像素再动作
    .restartDeadbandY = 15U,                        //Y 重启阈值：停止后超过 1.5 像素再动作
    .kpX = 100U,                                    //X 比例增益：左右轴响应强度
    .kpY = 50U,                                    //Y 比例增益：上下轴响应强度
    .kdX = 20U,                                      //X 微分增益：不用 D 就填 0
    .kdY = 20U,                                      //Y 微分增益：不用 D 就填 0
    .gainScale = STATICCONFIG_GAIN_SCALE_DEFAULT,   //增益缩放：默认除以 100
    .minSpeedX = 100U,                              //X 最小动作速度：单位 SPS
    .minSpeedY = 100U,                               //Y 最小动作速度：单位 SPS
    .maxSpeedX = 1500U,                            //X 最大速度：单位 SPS
    .maxSpeedY = 1500U,                            //Y 最大速度：单位 SPS
    .offsetX = -160,                                   //X 安装补偿：正值让点向右偏
    .offsetY = 80,                                  //Y 安装补偿：正值让点向下偏
    .useCircleError = 1U                            //误差来源：1 取第 3/4 个数
};

static const StaticConfigGimbalTask g_taskMidCenter = {
    .name = "mid_center",                          //调试名：中距离 + 矩形中心
    .distance = STATICCONFIG_DISTANCE_MID,          //距离档：中距离
    .mode = STATICCONFIG_MODE_CENTER,               //模式：使用矩形中心误差
    .turnAngleMinDeg = 40,                          //角度下限：中距离段起点
    .turnAngleMaxDeg = 9999,                         //角度上限：中距离段终点
    .deadbandX = 10U,                               //X 死区：1 像素
    .deadbandY = 10U,                               //Y 死区：1 像素
    .restartDeadbandX = 15U,                        //X 重启阈值：停止后超过 1.5 像素再动作
    .restartDeadbandY = 15U,                        //Y 重启阈值：停止后超过 1.5 像素再动作
    .kpX = 200U,                                    //X 比例增益：左右轴响应强度
    .kpY = 100U,                                    //Y 比例增益：上下轴响应强度
    .kdX = 0U,                                      //X 微分增益：不用 D 就填 0
    .kdY = 0U,                                      //Y 微分增益：不用 D 就填 0
    .gainScale = STATICCONFIG_GAIN_SCALE_DEFAULT,   //增益缩放：默认除以 100
    .minSpeedX = 200U,                              //X 最小动作速度：单位 SPS
    .minSpeedY = 100U,                               //Y 最小动作速度：单位 SPS
    .maxSpeedX = 10000U,                            //X 最大速度：单位 SPS
    .maxSpeedY = 10000U,                            //Y 最大速度：单位 SPS
    .offsetX = -170,                                   //X 安装补偿：正值让点向右偏
    .offsetY = 120,                                  //Y 安装补偿：正值让点向下偏
    .useCircleError = 0U                            //误差来源：0 取第 1/2 个数
};

static const StaticConfigGimbalTask g_taskMidCircle = {
    .name = "mid_circle",                          //调试名：中距离 + 圆点
    .distance = STATICCONFIG_DISTANCE_MID,          //距离档：中距离
    .mode = STATICCONFIG_MODE_CIRCLE,               //模式：使用圆点误差
    .turnAngleMinDeg = 61,                          //角度下限：中距离段起点
    .turnAngleMaxDeg = 140,                         //角度上限：中距离段终点
    .deadbandX = 10U,                               //X 死区：1 像素
    .deadbandY = 10U,                               //Y 死区：1 像素
    .restartDeadbandX = 15U,                        //X 重启阈值：停止后超过 1.5 像素再动作
    .restartDeadbandY = 15U,                        //Y 重启阈值：停止后超过 1.5 像素再动作
    .kpX = 200U,                                    //X 比例增益：左右轴响应强度
    .kpY = 100U,                                    //Y 比例增益：上下轴响应强度
    .kdX = 0U,                                      //X 微分增益：不用 D 就填 0
    .kdY = 0U,                                      //Y 微分增益：不用 D 就填 0
    .gainScale = STATICCONFIG_GAIN_SCALE_DEFAULT,   //增益缩放：默认除以 100
    .minSpeedX = 100U,                              //X 最小动作速度：单位 SPS
    .minSpeedY = 30U,                               //Y 最小动作速度：单位 SPS
    .maxSpeedX = 10000U,                            //X 最大速度：单位 SPS
    .maxSpeedY = 10000U,                            //Y 最大速度：单位 SPS
    .offsetX = -170,                                   //X 安装补偿：正值让点向右偏
    .offsetY = 110,                                  //Y 安装补偿：正值让点向下偏
    .useCircleError = 1U                            //误差来源：1 取第 3/4 个数
};

static const StaticConfigGimbalTask g_taskFarCenter = {
    .name = "far_center",                          //调试名：远距离 + 矩形中心
    .distance = STATICCONFIG_DISTANCE_FAR,          //距离档：远距离
    .mode = STATICCONFIG_MODE_CENTER,               //模式：使用矩形中心误差
    .turnAngleMinDeg = 141,                         //角度下限：远距离段起点
    .turnAngleMaxDeg = 32767,                       //角度上限：远距离段终点
    .deadbandX = 10U,                               //X 死区：1 像素
    .deadbandY = 10U,                               //Y 死区：1 像素
    .restartDeadbandX = 15U,                        //X 重启阈值：停止后超过 1.5 像素再动作
    .restartDeadbandY = 15U,                        //Y 重启阈值：停止后超过 1.5 像素再动作
    .kpX = 200U,                                    //X 比例增益：左右轴响应强度
    .kpY = 100U,                                    //Y 比例增益：上下轴响应强度
    .kdX = 0U,                                      //X 微分增益：不用 D 就填 0
    .kdY = 0U,                                      //Y 微分增益：不用 D 就填 0
    .gainScale = STATICCONFIG_GAIN_SCALE_DEFAULT,   //增益缩放：默认除以 100
    .minSpeedX = 100U,                              //X 最小动作速度：单位 SPS
    .minSpeedY = 30U,                               //Y 最小动作速度：单位 SPS
    .maxSpeedX = 10000U,                            //X 最大速度：单位 SPS
    .maxSpeedY = 10000U,                            //Y 最大速度：单位 SPS
    .offsetX = -170,                                   //X 安装补偿：正值让点向右偏
    .offsetY = 130,                                 //Y 安装补偿：正值让点向下偏
    .useCircleError = 0U                            //误差来源：0 取第 1/2 个数
};

static const StaticConfigGimbalTask g_taskFarCircle = {
    .name = "far_circle",                          //调试名：远距离 + 圆点
    .distance = STATICCONFIG_DISTANCE_FAR,          //距离档：远距离
    .mode = STATICCONFIG_MODE_CIRCLE,               //模式：使用圆点误差
    .turnAngleMinDeg = 141,                         //角度下限：远距离段起点
    .turnAngleMaxDeg = 32767,                       //角度上限：远距离段终点
    .deadbandX = 5U,                               //X 死区：1 像素
    .deadbandY = 5U,                               //Y 死区：1 像素
    .restartDeadbandX = 10U,                        //X 重启阈值：停止后超过 1.0 像素再动作
    .restartDeadbandY = 10U,                        //Y 重启阈值：停止后超过 1.0 像素再动作
    .kpX = 200U,                                    //X 比例增益：左右轴响应强度
    .kpY = 100U,                                    //Y 比例增益：上下轴响应强度
    .kdX = 0U,                                      //X 微分增益：不用 D 就填 0
    .kdY = 0U,                                      //Y 微分增益：不用 D 就填 0
    .gainScale = STATICCONFIG_GAIN_SCALE_DEFAULT,   //增益缩放：默认除以 100
    .minSpeedX = 100U,                              //X 最小动作速度：单位 SPS
    .minSpeedY = 30U,                               //Y 最小动作速度：单位 SPS
    .maxSpeedX = 10000U,                            //X 最大速度：单位 SPS
    .maxSpeedY = 10000U,                            //Y 最大速度：单位 SPS
    .offsetX = -170,                                   //X 安装补偿：正值让点向右偏
    .offsetY = 120,                                  //Y 安装补偿：正值让点向下偏
    .useCircleError = 1U                            //误差来源：1 取第 3/4 个数
};

static const StaticConfigGimbalTask *const g_tasks[STATICCONFIG_TASK_COUNT] = {
    &g_taskNearCenter,     //近距离 + 矩形中心
    &g_taskNearCircle,     //近距离 + 圆点
    &g_taskMidCenter,      //中距离 + 矩形中心
    &g_taskMidCircle,      //中距离 + 圆点
    &g_taskFarCenter,      //远距离 + 矩形中心
    &g_taskFarCircle       //远距离 + 圆点
};

static StaticConfigTaskId g_activeTask = STATICCONFIG_TASK_NEAR_CENTER;  //当前 active 参数，Init 后会改成默认任务

static uint8_t StaticConfig_IsValidTask(StaticConfigTaskId taskId)  //检查任务编号是否在 0~COUNT-1 内
{
    return ((uint8_t)taskId < (uint8_t)STATICCONFIG_TASK_COUNT) ? 1U : 0U;  //合法返回 1，非法返回 0
}

void StaticConfig_Init(void)
{
    g_activeTask = STATICCONFIG_TASK_NEAR_CENTER;  //上电默认使用近距离矩形中心误差
}

StaticConfigTaskId StaticConfig_GetActiveTask(void)
{
    return g_activeTask;  //返回当前正在使用的六套参数编号
}

void StaticConfig_SetActiveTask(StaticConfigTaskId taskId)
{
    if (StaticConfig_IsValidTask(taskId) != 0U) {  //只接受合法任务编号
        g_activeTask = taskId;                     //切换 active 参数
    }
}

const StaticConfigGimbalTask *StaticConfig_GetTask(StaticConfigTaskId taskId)
{
    if (StaticConfig_IsValidTask(taskId) == 0U) {  //传入非法编号时走兜底参数
        taskId = STATICCONFIG_TASK_NEAR_CENTER;    //兜底读取近距离中心参数
    }
    return g_tasks[(uint8_t)taskId];               //返回任务编号对应的结构体指针
}

const StaticConfigGimbalTask *StaticConfig_GetActiveGimbal(void)
{
    return StaticConfig_GetTask(g_activeTask);  //给 gimbal.c 读取当前闭环参数
}

StaticConfigTaskId StaticConfig_GetTaskFor(StaticConfigDistance distance,
    StaticConfigMode mode)
{
    if (distance == STATICCONFIG_DISTANCE_NEAR) {  //近距离档
        return (mode == STATICCONFIG_MODE_CIRCLE) ?
            STATICCONFIG_TASK_NEAR_CIRCLE : STATICCONFIG_TASK_NEAR_CENTER;  //按模式选中心/圆点
    }
    if (distance == STATICCONFIG_DISTANCE_MID) {   //中距离档
        return (mode == STATICCONFIG_MODE_CIRCLE) ?
            STATICCONFIG_TASK_MID_CIRCLE : STATICCONFIG_TASK_MID_CENTER;    //按模式选中心/圆点
    }
    return (mode == STATICCONFIG_MODE_CIRCLE) ?
        STATICCONFIG_TASK_FAR_CIRCLE : STATICCONFIG_TASK_FAR_CENTER;        //其它情况按远距离档处理
}

void StaticConfig_SetActiveByDistanceMode(StaticConfigDistance distance,
    StaticConfigMode mode)
{
    StaticConfig_SetActiveTask(StaticConfig_GetTaskFor(distance, mode));  //距离档 + 模式直接切换 active
}

void StaticConfig_UpdateByTurnAngleDeg(int16_t turnAngleDeg,
    StaticConfigMode mode)
{
    uint8_t i;                              //遍历六套参数用的下标
    const StaticConfigGimbalTask *task;     //当前检查的任务参数

    for (i = 0U; i < (uint8_t)STATICCONFIG_TASK_COUNT; ++i) {  //逐个检查六套参数
        task = g_tasks[i];                                     //取出当前任务
        if ((task->mode == mode) &&
            (turnAngleDeg >= task->turnAngleMinDeg) &&
            (turnAngleDeg <= task->turnAngleMaxDeg)) {         //模式相同且角度落在范围内
            g_activeTask = (StaticConfigTaskId)i;               //命中后切换 active 参数
            return;                                            //找到就退出，避免后面覆盖
        }
    }
}
