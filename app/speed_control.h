#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdint.h>

/* SpeedControl_Init：初始化速度闭环状态，不负责初始化电机和编码器硬件。 */
void SpeedControl_Init(void);

/* SpeedControl_SetTarget：设置左右轮目标速度命令，单位沿用上层的有符号命令值。 */
void SpeedControl_SetTarget(int16_t left, int16_t right);

/* SpeedControl_Stop：清空速度目标、积分和输出，并立即停车。 */
void SpeedControl_Stop(void);

/* SpeedControl_Task：周期读取编码器并刷新左右电机 PWM。 */
void SpeedControl_Task(void);

/* SpeedControl_GetLeftActual：读取左轮最近一个控制周期的编码器增量。 */
int16_t SpeedControl_GetLeftActual(void);

/* SpeedControl_GetRightActual：读取右轮最近一个控制周期的编码器增量。 */
int16_t SpeedControl_GetRightActual(void);

/* SpeedControl_GetLeftOutput：读取左轮最近一次闭环输出 PWM。 */
int16_t SpeedControl_GetLeftOutput(void);

/* SpeedControl_GetRightOutput：读取右轮最近一次闭环输出 PWM。 */
int16_t SpeedControl_GetRightOutput(void);

#endif
