#ifndef MOTOR_NO_YAW_H
#define MOTOR_NO_YAW_H

#include <stdint.h>

typedef enum {
    MOTOR_NO_YAW_STATE_IDLE = 0,
    MOTOR_NO_YAW_STATE_LINE,
    MOTOR_NO_YAW_STATE_TURN_APPROACH,
    MOTOR_NO_YAW_STATE_TURN_RIGHT,
    MOTOR_NO_YAW_STATE_TURN_LEFT,
    MOTOR_NO_YAW_STATE_STOP
} MotorNoYawState;

/* MotorNoYaw_Init：初始化 NO YAW 灰度触发/转向状态，默认不输出底盘命令。 */
void MotorNoYaw_Init(void);

/* MotorNoYaw_Start：开始灰度循迹、侧边三路触发缓慢转向。 */
void MotorNoYaw_Start(void);

/* MotorNoYaw_Stop：停止无 yaw 循迹并恢复底盘 EN 默认状态。 */
void MotorNoYaw_Stop(void);

/* MotorNoYaw_Task：执行一轮灰度循迹、触发转向或转向退出。 */
void MotorNoYaw_Task(void);

/* MotorNoYaw_FastSample：NO YAW 运行时快速补采灰度，只抓直角触发/退出。 */
void MotorNoYaw_FastSample(void);

/* MotorNoYaw_LogStopReason：低优先级打印异常停车原因，正常运行时不打印。 */
void MotorNoYaw_LogStopReason(void);

/* MotorNoYaw_IsRunning：返回无 yaw 循迹是否仍在运行。 */
uint8_t MotorNoYaw_IsRunning(void);

/* MotorNoYaw_GetState/GetStateName：读取当前状态，供 OLED 和日志显示。 */
MotorNoYawState MotorNoYaw_GetState(void);
const char *MotorNoYaw_GetStateName(void);

/* MotorNoYaw_GetPhase：读取转向次数 flag % 4。 */
uint8_t MotorNoYaw_GetPhase(void);

/* MotorNoYaw_GetDigitalMask：读取最近一轮灰度 bitmask。 */
uint8_t MotorNoYaw_GetDigitalMask(void);

/* MotorNoYaw_GetLineError：读取最近一轮加权循迹误差。 */
int16_t MotorNoYaw_GetLineError(void);

#endif
