#ifndef GIMBAL_TEST_H
#define GIMBAL_TEST_H

#include <stdint.h>

/* GimbalTest_Init：初始化云台视觉测试状态，不启用电机输出。 */
void GimbalTest_Init(void);

/*
 * GimbalTest_Start：进入云台视觉测试。
 * 说明：清空 UART3/Link 历史缓存，启用云台闭环，等待视觉坐标输入。
 */
void GimbalTest_Start(void);

/* GimbalTest_Stop：退出云台测试并停止云台。 */
void GimbalTest_Stop(void);

/*
 * GimbalTest_Task：读取 Link 收到的视觉数据并更新云台。
 * 视觉格式："centerDx,centerDy;circleDx,circleDy"，单位由解析器转成 0.1 像素。
 * 选择中心误差还是圆点误差由 CAR_GIMBAL_TEST_USE_CIRCLE_ERROR 配置。
 */
void GimbalTest_Task(void);

/* GimbalTest_IsRunning：当前是否处于 Vision Test 页面对应的运行状态。 */
uint8_t GimbalTest_IsRunning(void);

/* GimbalTest_HasVision：是否已经收到并应用过至少一帧有效视觉数据。 */
uint8_t GimbalTest_HasVision(void);

/* GimbalTest_GetFrameCount：返回有效视觉帧计数，便于串口调试。 */
uint32_t GimbalTest_GetFrameCount(void);

/* GimbalTest_GetBadFrameCount：返回坏帧计数，便于判断视觉协议是否对上。 */
uint32_t GimbalTest_GetBadFrameCount(void);

#endif
