#ifndef RTOS_APP_H
#define RTOS_APP_H

#include <stdint.h>

#include "state_machine.h"

/* 创建全部静态 RTOS 对象并启动调度器；正常情况下不会返回。 */
void RtosApp_StartScheduler(void);

/* 任务上下文使用的唤醒接口。通知只表示“需要处理”，不携带业务数据。 */
void RtosApp_NotifyGimbal(void);
void RtosApp_NotifyMission(void);
void RtosApp_NotifyUi(void);

/* Queue one state-machine event from a task such as the H7 command service. */
uint8_t RtosApp_PostEvent(CarEvent event);

/* 中断上下文专用接口；内部使用 FromISR API，并按需在退出中断时切换任务。 */
void RtosApp_NotifyControlFromISR(void);
void RtosApp_NotifyGimbalFromISR(void);
void RtosApp_NotifyInputFromISR(void);

#endif
