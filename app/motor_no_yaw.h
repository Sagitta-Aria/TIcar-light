#ifndef MOTOR_NO_YAW_H
#define MOTOR_NO_YAW_H

#include <stdint.h>

/* 灰度循迹内部状态；只由MotorNoYaw_Task和快采样事件推进。 */
typedef enum {
    MOTOR_NO_YAW_STATE_IDLE = 0,      /* 尚未启动，不输出循迹命令。 */
    MOTOR_NO_YAW_STATE_LINE,          /* 普通灰度加权循迹。 */
    MOTOR_NO_YAW_STATE_TURN_APPROACH, /* 已确认直角，继续向前靠近弯心。 */
    MOTOR_NO_YAW_STATE_TURN_RIGHT,    /* 正在执行右侧强转。 */
    MOTOR_NO_YAW_STATE_TURN_EXIT,     /* S1/S7回线后的短暂向前摆正。 */
    MOTOR_NO_YAW_STATE_TURN_LEFT,     /* 正在执行左侧强转。 */
    MOTOR_NO_YAW_STATE_STOP           /* 丢线/转向超时导致的异常停车。 */
} MotorNoYawState;

/* 异常停车原因；主动调用MotorNoYaw_Stop不会伪造异常原因。 */
typedef enum {
    MOTOR_NO_YAW_STOP_NONE = 0,      /* 无异常。 */
    MOTOR_NO_YAW_STOP_LINE_LOST,     /* 普通循迹丢线超过配置时间。 */
    MOTOR_NO_YAW_STOP_TURN_TIMEOUT   /* 强转等待S1/S7回线超过安全上限。 */
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

/* TIMG0每100us调用；内部按board_config的语义间隔采样，形成事件时返回1。 */
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
