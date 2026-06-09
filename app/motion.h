#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

/* Motion_Stop：停止底盘和云台底层电机速度；用于状态切换或异常停车。 */
void Motion_Stop(void);

/* Motion_SetChassisCommand：直接设置底盘左右 SPS，正负号表示方向。 */
void Motion_SetChassisCommand(int16_t leftSpeedSps, int16_t rightSpeedSps);

/* Motion_Forward：底盘两侧同向前进，speedSps 为 step/s。 */
void Motion_Forward(uint16_t speedSps);

/* Motion_Backward：底盘两侧同向后退，speedSps 为 step/s。 */
void Motion_Backward(uint16_t speedSps);

/* Motion_TurnLeft：底盘原地左转，左轮后退、右轮前进。 */
void Motion_TurnLeft(uint16_t speedSps);

/* Motion_TurnRight：底盘原地右转，左轮前进、右轮后退。 */
void Motion_TurnRight(uint16_t speedSps);

#endif
