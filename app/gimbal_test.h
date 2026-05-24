#ifndef GIMBAL_TEST_H
#define GIMBAL_TEST_H

#include <stdint.h>

/* GimbalTest_Init：初始化云台测试状态。 */
void GimbalTest_Init(void);

/* GimbalTest_Start：进入云台测试，清空视觉缓存并启用云台闭环。 */
void GimbalTest_Start(void);

/* GimbalTest_Stop：退出云台测试并停止云台。 */
void GimbalTest_Stop(void);

/* GimbalTest_Task：读取 Link 收到的视觉数据并更新云台目标。 */
void GimbalTest_Task(void);

uint8_t GimbalTest_IsRunning(void);
uint8_t GimbalTest_HasVision(void);
uint32_t GimbalTest_GetFrameCount(void);
uint32_t GimbalTest_GetBadFrameCount(void);

#endif
