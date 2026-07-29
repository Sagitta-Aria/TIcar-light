#ifndef APP_H
#define APP_H

#include "state_machine.h"

/* App_Init：初始化应用层模块；Board_Init 成功后由 main 调用一次。 */
void App_Init(void);

/* App_Task：保留的裸机兼容入口，正常构建由rtos_app.c分别调度各Step函数。 */
void App_Task(void);

/* Input任务入口：消费一次按键状态并返回要交给状态机的事件。 */
CarEvent App_InputStep(void);

/* 返回上一轮App_InputStep是否产生事件；供Input任务决定是否唤醒UI。 */
uint8_t App_InputHadEvent(void);

/* K1/K2仍按住或有待处理事件时返回1，决定Input任务是否保持1ms轮询。 */
uint8_t App_InputIsActive(void);

/* Mission任务上下文中分发状态机事件；不要从GPIO中断直接调用。 */
void App_MissionDispatch(CarEvent event);

/* Mission任务的一次业务推进；不承担10ms底盘闭环。 */
void App_MissionStep(void);

/* Comm任务的一次5ms维护：视觉链路、Task5命令和低频通信。 */
void App_CommStep(void);

/* Gimbal任务入口：处理视觉帧、H7反馈和JY61姿态补偿。 */
void App_GimbalStep(void);

/* 仅解析视觉输入；供Task4转向时丢帧控制但保持UART队列畅通。 */
void App_VisionInputStep(void);

/* UI任务入口：仅在任务上下文刷新H7 LCD，禁止在ISR内调用。 */
void App_UiStep(void);

/* 低频板级维护并返回UI心跳；Watchdog任务用返回值判断界面是否存活。 */
uint8_t App_HousekeepingStep(void);

#endif
