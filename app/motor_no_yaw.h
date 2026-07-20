#ifndef MOTOR_NO_YAW_H
#define MOTOR_NO_YAW_H

#include <stdint.h>

typedef enum {
    MOTOR_NO_YAW_STATE_IDLE = 0,
    MOTOR_NO_YAW_STATE_LINE,
    MOTOR_NO_YAW_STATE_TURN_APPROACH,
    MOTOR_NO_YAW_STATE_TURN_RIGHT,
    MOTOR_NO_YAW_STATE_TURN_EXIT,
    MOTOR_NO_YAW_STATE_TURN_LEFT,
    MOTOR_NO_YAW_STATE_STOP
} MotorNoYawState;

typedef enum {
    MOTOR_NO_YAW_STOP_NONE = 0,
    MOTOR_NO_YAW_STOP_LINE_LOST,
    MOTOR_NO_YAW_STOP_TURN_TIMEOUT
} MotorNoYawStopReason;

/* MotorNoYaw_Init：初始化 NO YAW 灰度触发/转向状态，默认不输出底盘命令。 */
void MotorNoYaw_Init(void);

/* MotorNoYaw_Start：开始灰度循迹；姿态角只辅助粗略减速，灰度决定出弯。 */
void MotorNoYaw_Start(void);

/* MotorNoYaw_StartMission4：使用完全独立的 Task4 循迹与强转配置。 */
void MotorNoYaw_StartMission4(void);

/* MotorNoYaw_Stop：停止无 yaw 循迹并将底盘PWM目标清零。 */
void MotorNoYaw_Stop(void);

/* MotorNoYaw_Task：每10ms执行一轮灰度循迹、触发转向或转向退出。 */
void MotorNoYaw_Task(void);

/* 按当前编译算法和 Task1 参数应用一拍灰度命令；Task5 gray 联调使用。 */
uint8_t MotorNoYaw_ApplyTask1LineCommand(uint8_t digitalMask);

/* MotorNoYaw_HandleFastEvent：消费100us快采样形成的左右入弯和回线请求。 */
void MotorNoYaw_HandleFastEvent(void);

/* MotorNoYaw_TimerSample：TIMG0 100us灰度中断里采样；形成语义事件时返回1。 */
uint8_t MotorNoYaw_TimerSample(void);

/* MotorNoYaw_IsRunning：返回无 yaw 循迹是否仍在运行。 */
uint8_t MotorNoYaw_IsRunning(void);

/* MotorNoYaw_GetState/GetStateName：读取当前状态，供 OLED 和日志显示。 */
MotorNoYawState MotorNoYaw_GetState(void);
const char *MotorNoYaw_GetStateName(void);

/* MotorNoYaw_GetStopReason：读取最近一次异常停车原因；新任务启动时清零。 */
MotorNoYawStopReason MotorNoYaw_GetStopReason(void);

/* MotorNoYaw_GetPhase：读取转向次数 flag % 4。 */
uint8_t MotorNoYaw_GetPhase(void);

/* MotorNoYaw_GetTurnCount：读取已完成的总转向次数。 */
uint32_t MotorNoYaw_GetTurnCount(void);

/* MotorNoYaw_GetDigitalMask：读取最近一轮灰度 bitmask。 */
uint8_t MotorNoYaw_GetDigitalMask(void);

/* MotorNoYaw_GetLineError：读取最近一轮加权循迹误差。 */
int16_t MotorNoYaw_GetLineError(void);

#endif
