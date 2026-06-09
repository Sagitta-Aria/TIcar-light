#ifndef MOTOR_ENABLE_H
#define MOTOR_ENABLE_H

#include <stdint.h>

/*
 * MotorEnable_Init：初始化四路步进驱动器 EN 输出的默认状态。
 * 说明：GPIO 方向在 generated/ti_msp_dl_config.c 中配置，这里只写默认电平。
 */
void MotorEnable_Init(void);

/*
 * MotorEnable_SetAll：同时设置四个步进驱动器 EN。
 * enabled 非 0 表示使能，0 表示释放/禁用；实际高低电平由 board_config.h 决定。
 */
void MotorEnable_SetAll(uint8_t enabled);

/*
 * MotorEnable_SetGimbal：只设置云台两路 EN。
 * 使用场景：视觉闭环或云台单独测试，避免误使能底盘驱动器。
 */
void MotorEnable_SetGimbal(uint8_t enabled);

/* MotorEnable_IsAllEnabled：读取四路 EN 是否都处于使能状态。 */
uint8_t MotorEnable_IsAllEnabled(void);

/* MotorEnable_GetActiveLevelName：返回当前配置的 EN 有效电平文本，供 OLED 显示。 */
const char *MotorEnable_GetActiveLevelName(void);

#endif
