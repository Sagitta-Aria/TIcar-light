#ifndef MOTOR_ENABLE_H
#define MOTOR_ENABLE_H

#include <stdint.h>

/*
 * MotorEnable_Init：保留原状态机接口；新板云台EN由硬件固定有效。
 */
void MotorEnable_Init(void);

/*
 * MotorEnable_SetAll：保留云台EN请求状态；当前硬件EN必须固定有效。
 * enabled非0表示状态机请求使能，本接口不直接驱动GPIO。
 */
void MotorEnable_SetAll(uint8_t enabled);

/*
 * MotorEnable_SetChassis：编码底盘无独立EN，保留为空操作兼容接口。
 */
void MotorEnable_SetChassis(uint8_t enabled);

/*
 * MotorEnable_SetGimbal：记录云台使能请求，实际EN由硬件固定。
 */
void MotorEnable_SetGimbal(uint8_t enabled);

/* MotorEnable_IsAllEnabled：读取最近一次云台EN请求状态。 */
uint8_t MotorEnable_IsAllEnabled(void);

/* MotorEnable_GetActiveLevelName：返回当前配置的 EN 有效电平文本，供 OLED 显示。 */
const char *MotorEnable_GetActiveLevelName(void);

#endif
