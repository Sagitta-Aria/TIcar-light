#ifndef MOTOR_NO_YAW_H
#define MOTOR_NO_YAW_H

#include <stdint.h>

typedef enum {
    MOTOR_NO_YAW_STATE_IDLE = 0,
    MOTOR_NO_YAW_STATE_LINE,
    MOTOR_NO_YAW_STATE_TURN_RIGHT,
    MOTOR_NO_YAW_STATE_TURN_LEFT,
    MOTOR_NO_YAW_STATE_STOP
} MotorNoYawState;

/* MotorNoYaw_Init：初始化 NO YAW 灰度循迹/开环转角状态，默认不输出底盘命令。 */
void MotorNoYaw_Init(void);

/* MotorNoYaw_Start：开始灰度循迹直行/定时右转，清空状态并启用底盘。 */
void MotorNoYaw_Start(void);

/* MotorNoYaw_Stop：停止无 yaw 循迹并恢复底盘 EN 默认状态。 */
void MotorNoYaw_Stop(void);

/* MotorNoYaw_Task：执行一轮灰度循迹直行和定时原地右转。 */
void MotorNoYaw_Task(void);

/* MotorNoYaw_IsRunning：返回无 yaw 循迹是否仍在运行。 */
uint8_t MotorNoYaw_IsRunning(void);

/* MotorNoYaw_GetState/GetStateName：读取当前状态，供 OLED 和日志显示。 */
MotorNoYawState MotorNoYaw_GetState(void);
const char *MotorNoYaw_GetStateName(void);

/* MotorNoYaw_GetPhase：读取转向次数 flag % 4。 */
uint8_t MotorNoYaw_GetPhase(void);

/* MotorNoYaw_GetTravelSteps：读取当前开环直行段的平均 STEP。 */
uint32_t MotorNoYaw_GetTravelSteps(void);

/* MotorNoYaw_GetDigitalMask：读取最近一轮灰度 bitmask。 */
uint8_t MotorNoYaw_GetDigitalMask(void);

/* MotorNoYaw_GetLineError：读取最近一轮加权循迹误差。 */
int16_t MotorNoYaw_GetLineError(void);

#endif
