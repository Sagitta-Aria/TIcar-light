#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

/*
 * GimbalPoint：视觉坐标点。
 * 使用场景：目标点和当前识别点都用同一坐标系，例如摄像头像素坐标。
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

/* Gimbal_SetTarget：单独设置目标位置。 */
void Gimbal_SetTarget(int16_t x, int16_t y);

/* Gimbal_SetCurrent：单独设置当前位置。 */
void Gimbal_SetCurrent(int16_t x, int16_t y);

/*
 * Gimbal_UpdateFromVision：用一帧视觉数据更新目标点和当前点。
 * 使用场景：视觉串口解析完成后调用。
 * 说明：本函数不阻塞、不等待串口；真正输出由 Gimbal_Task 周期执行。
 */
void Gimbal_UpdateFromVision(int16_t targetX, int16_t targetY,
    int16_t currentX, int16_t currentY);

/* Gimbal_Task：云台周期控制任务。 */
void Gimbal_Task(void);

/* Gimbal_Stop：停止两个云台轴，不影响底盘。 */
void Gimbal_Stop(void);

/* Gimbal_GetErrorX/Y：读取最近一帧视觉误差，等于 target - current。 */
int16_t Gimbal_GetErrorX(void);
int16_t Gimbal_GetErrorY(void);

/* Gimbal_GetCommandX/Y：读取最近一次输出到云台轴的有符号命令。 */
int16_t Gimbal_GetCommandX(void);
int16_t Gimbal_GetCommandY(void);

#endif
