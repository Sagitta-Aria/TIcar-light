#ifndef APP_H
#define APP_H

#include "state_machine.h"

/* App_Init：初始化应用层模块；Board_Init 成功后由 main 调用一次。 */
void App_Init(void);

/* App_Task：保留的裸机兼容入口，正常构建由rtos_app.c分别调度各Step函数。 */
void App_Task(void);

CarEvent App_InputStep(void);
uint8_t App_InputHadEvent(void);
uint8_t App_InputIsActive(void);
void App_MissionDispatch(CarEvent event);
void App_MissionStep(void);
void App_CommStep(void);
void App_GimbalStep(void);
/* 仅解析视觉输入；供Task4转向时丢帧控制但保持UART队列畅通。 */
void App_VisionInputStep(void);
void App_UiStep(void);
uint8_t App_HousekeepingStep(void);

#endif
