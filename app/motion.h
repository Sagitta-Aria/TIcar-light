#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

/* Motion_Stop：停止底盘和云台底层电机速度；用于状态切换或异常停车。 */
void Motion_Stop(void);

/* Motion_SetChassisCommand：设置底盘左右目标 CPS，正负号表示方向。 */
void Motion_SetChassisCommand(int16_t leftSpeedCps, int16_t rightSpeedCps);

/* 设置左右目标 encoder count/控制周期；用于 NO YAW 与 Task5 共用速度刻度。 */
void Motion_SetChassisPeriodCommand(int16_t leftCounts,
    int16_t rightCounts);

/* Motion_Forward：底盘两侧以相同目标 CPS 前进。 */
void Motion_Forward(uint16_t speedCps);

/* Motion_Backward：底盘两侧以相同目标 CPS 后退。 */
void Motion_Backward(uint16_t speedCps);

/* Motion_TurnLeft：底盘原地左转，左轮后退、右轮前进。 */
void Motion_TurnLeft(uint16_t speedCps);

/* Motion_TurnRight：底盘原地右转，左轮前进、右轮后退。 */
void Motion_TurnRight(uint16_t speedCps);

#endif
