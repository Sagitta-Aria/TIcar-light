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

/* MotorNoYaw_Init：初始化 NO YAW 灰度触发/转向状态，默认不输出底盘命令。 */
void MotorNoYaw_Init(void);

/* MotorNoYaw_Start：开始灰度循迹；姿态角只辅助粗略减速，灰度决定出弯。 */
void MotorNoYaw_Start(void);

/* 兼容的 Task4 profile 入口；当前正式 Task4 直接调用 MotorNoYaw_Start。 */
void MotorNoYaw_StartMission4(void);

/* MotorNoYaw_Stop：停止无 yaw 循迹并将底盘PWM目标清零。 */
void MotorNoYaw_Stop(void);

/* MotorNoYaw_Task：每20ms执行一轮灰度循迹、触发转向或转向退出。 */
void MotorNoYaw_Task(void);

/*
 * MotorNoYaw_CalculateTask1LineCommand：按 Task1 参数把灰度 mask 换算成
 * 左右 encoder count/控制周期，与 Task5 target 命令同单位。
 * 返回0表示当前没有有效灰度输入，输出速度同时置零。
 */
uint8_t MotorNoYaw_CalculateTask1LineCommand(uint8_t digitalMask,
    int16_t *leftTargetCounts, int16_t *rightTargetCounts);

/* MotorNoYaw_HandleFastEvent：消费100us快采样形成的左右入弯和回线请求。 */
void MotorNoYaw_HandleFastEvent(void);

/* MotorNoYaw_TimerSample：TIMG0 100us灰度中断里采样；形成语义事件时返回1。 */
uint8_t MotorNoYaw_TimerSample(void);

/* MotorNoYaw_IsRunning：返回无 yaw 循迹是否仍在运行。 */
uint8_t MotorNoYaw_IsRunning(void);

/* MotorNoYaw_GetState/GetStateName：读取当前状态，供 OLED 和日志显示。 */
MotorNoYawState MotorNoYaw_GetState(void);
const char *MotorNoYaw_GetStateName(void);

/* MotorNoYaw_GetPhase：读取转向次数 flag % 4。 */
uint8_t MotorNoYaw_GetPhase(void);

/* MotorNoYaw_GetTurnCount：读取已完成的总转向次数。 */
uint32_t MotorNoYaw_GetTurnCount(void);

/* MotorNoYaw_GetDigitalMask：读取最近一轮灰度 bitmask。 */
uint8_t MotorNoYaw_GetDigitalMask(void);

/* MotorNoYaw_GetLineError：读取最近一轮加权循迹误差。 */
int16_t MotorNoYaw_GetLineError(void);

#endif
