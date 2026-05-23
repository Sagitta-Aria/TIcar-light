#ifndef JY61P_H
#define JY61P_H

#include <stdint.h>

/* JY61P_Init：初始化串口接收中断，等待 JY61P 姿态数据。 */
void JY61P_Init(void);

/* JY61P_Task：JY61P 后台任务入口，当前解析在串口中断内完成。 */
void JY61P_Task(void);

/* JY61P_HandleUARTInterrupt：处理 JY61P 串口接收中断并解析 yaw。 */
void JY61P_HandleUARTInterrupt(void);

/* JY61P_SendByte：向 JY61P 模块发送 1 个字节。 */
void JY61P_SendByte(uint8_t data);

/* JY61P_SendBytes：向 JY61P 模块发送一段连续数据。 */
void JY61P_SendBytes(const uint8_t *data, uint16_t length);

/* JY61P_SetYawDeg：保存当前航向角，后续解析模块可直接写入。 */
void JY61P_SetYawDeg(int16_t yawDeg);

/* JY61P_GetYawDeg：读取当前保存的航向角。 */
int16_t JY61P_GetYawDeg(void);

/* JY61P_HasYaw：判断是否已经收到过有效航向角。 */
uint8_t JY61P_HasYaw(void);

#endif
