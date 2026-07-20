#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

/*
 * GimbalPoint：视觉坐标点。
 * 使用场景：目标点和当前识别点都用同一坐标系，当前单位为 0.1 像素。
 */
typedef struct {
    int16_t x;
    int16_t y;
} GimbalPoint;

/* Gimbal_Init：初始化二维云台控制状态，默认不输出脉冲。 */
void Gimbal_Init(void);

/*
 * Gimbal_SetEnabled：启停云台闭环。
 * 说明：关闭时只停止两个云台电机，不影响底盘电机。
 */
void Gimbal_SetEnabled(uint8_t enabled);

/* Gimbal_IsEnabled：读取云台闭环是否启用。 */
uint8_t Gimbal_IsEnabled(void);

/*
 * 开关视觉误差对云台的控制输出；关闭期间帧可继续解析，但不更新yaw/pitch。
 * 重新打开后等待下一帧，不复用关闭前的误差或D/前馈状态。
 */
void Gimbal_SetVisionTrackingEnabled(uint8_t enabled);

/* 读取视觉误差当前是否允许驱动云台，供RTOS区分抢占与固定周期路径。 */
uint8_t Gimbal_IsVisionTrackingEnabled(void);

/* Gimbal_NeedsTimeoutService：已有视觉帧时返回1，供任务设置掉线超时。 */
uint8_t Gimbal_NeedsTimeoutService(void);

/* Gimbal_SetTarget：单独设置目标位置，当前单位为 0.1 像素。 */
void Gimbal_SetTarget(int16_t x, int16_t y);

/* Gimbal_SetCurrent：单独设置当前位置，当前单位为 0.1 像素。 */
void Gimbal_SetCurrent(int16_t x, int16_t y);

/*
 * Gimbal_UpdateFromVision：用一帧视觉数据更新目标点和当前点。
 * 使用场景：视觉串口解析完成后调用。
 * 说明：本函数不阻塞、不等待串口；真正输出由 Gimbal_Task 收到通知后执行。
 */
void Gimbal_UpdateFromVision(int16_t targetX, int16_t targetY,
    int16_t currentX, int16_t currentY);

/*
 * Gimbal_UpdateFromLaserError：用视觉端发来的“激光点 - 目标点”误差更新云台。
 * 使用场景：视觉脚本发送 dx,dy，而不是 target/current 绝对坐标。
 * 说明：输入单位为 0.1 像素；视觉误差为 current - target，本模块内部会转成 target - current。
 */
void Gimbal_UpdateFromLaserError(int16_t laserMinusTargetX,
    int16_t laserMinusTargetY);

/*
 * Gimbal_UpdateFromCameraError：用视觉端发来的“目标点 - 当前点”误差更新云台。
 * 使用场景：CanMV 脚本发送 160-x,120-y 这类中心参考误差。
 * 说明：输入单位为 0.1 像素；符号已经等于控制内部的 target - current，不再取反。
 */
void Gimbal_UpdateFromCameraError(int16_t targetMinusCurrentX,
    int16_t targetMinusCurrentY);

/* Gimbal_Task：处理一次待执行控制或视觉超时。 */
void Gimbal_Task(void);

/*
 * Gimbal_SetYawFeedForward：给 yaw 轴叠加基础速度，符号为控制逻辑方向。
 * 使用场景：Task4 行进时让云台 yaw 随底盘持续慢速跟随。
 */
void Gimbal_SetYawFeedForward(int16_t speedSps);

/* 写入姿态环的电机方向补偿速度；只允许高优先级 Gimbal 任务调用。 */
void Gimbal_SetYawAttitudeCompensation(int16_t speedSps);

/* 写入视觉目标长度拟合出的yaw增益，Q1024；不缩放H7/JY61或固定yaw命令。 */
void Gimbal_SetVisionYawGainQ1024(uint16_t gainQ1024);

/* 读取当前视觉yaw拟合增益，1024表示1.0倍。 */
uint16_t Gimbal_GetVisionYawGainQ1024(void);

/* Task4首帧锁定后允许视觉超时触发左右摆动重搜；其他任务保持关闭。 */
void Gimbal_SetLostTargetSearchEnabled(uint8_t enabled);

/* Task4 当前正处于丢目标左右搜索时返回1。 */
uint8_t Gimbal_IsLostTargetSearchActive(void);

/* Gimbal_ResetRamp：Task4 临时覆盖结束后恢复两个云台轴的默认斜坡。 */
void Gimbal_ResetRamp(void);

/* Gimbal_Stop：停止两个云台轴，不影响底盘。 */
void Gimbal_Stop(void);

/* Gimbal_GetErrorX/Y：读取最近一帧视觉误差，等于 target - current，单位为 0.1 像素。 */
int16_t Gimbal_GetErrorX(void);
int16_t Gimbal_GetErrorY(void);

/* Gimbal_GetCommandX/Y：读取最近一次输出到云台轴的有符号 SPS。 */
int16_t Gimbal_GetCommandX(void);
int16_t Gimbal_GetCommandY(void);

/* 读取最近一次视觉误差趋势产生的前馈 SPS，不包含固定yaw或姿态补偿。 */
int16_t Gimbal_GetVisionFeedForwardX(void);
int16_t Gimbal_GetVisionFeedForwardY(void);

#endif
