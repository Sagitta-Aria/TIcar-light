#ifndef VISION_H
#define VISION_H

#include <stdint.h>

/* Vision_Init：初始化视觉输入状态，不启用解析。 */
void Vision_Init(void);

/* Vision_Start：清空 Link 缓存并开始解析视觉行。 */
void Vision_Start(void);

/* Vision_Stop：停止解析视觉行。 */
void Vision_Stop(void);

/* Vision_Task：从 Link 取完整行，解析后写入 Gimbal 视觉误差。 */
void Vision_Task(void);

/* Vision_IsRunning：读取视觉输入是否正在解析。 */
uint8_t Vision_IsRunning(void);

/* Vision_HasFrame：是否已经收到并应用过有效视觉帧。 */
uint8_t Vision_HasFrame(void);

/* Vision_GetFrameCount/GetBadFrameCount：读取有效帧和坏帧计数。 */
uint32_t Vision_GetFrameCount(void);
uint32_t Vision_GetBadFrameCount(void);

/* Vision_GetRawX/GetRawY：读取最近一次视觉误差，单位 0.1 像素。 */
int16_t Vision_GetRawX(void);
int16_t Vision_GetRawY(void);

/* 读取第五字段阶段标度，单位0.1；610表示61.0。 */
int16_t Vision_GetStageScaleX10(void);

/* 阶段标度严格大于60.0且已应用1.4倍yaw增益时返回1。 */
uint8_t Vision_IsYawBoostActive(void);

#endif
