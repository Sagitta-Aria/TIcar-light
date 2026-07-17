#ifndef RTOS_APP_H
#define RTOS_APP_H

void RtosApp_StartScheduler(void);
void RtosApp_NotifyGimbal(void);
void RtosApp_NotifyMission(void);
void RtosApp_NotifyUi(void);
void RtosApp_NotifyControlFromISR(void);
void RtosApp_NotifyGimbalFromISR(void);
void RtosApp_NotifyInputFromISR(void);

#endif
