#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/*
 * MotorId：四个闭环步进驱动器的逻辑编号。
 * 左右底盘优先保留在前两个编号，便于旧循迹代码继续沿用 left/right 语义。
 */
typedef enum {
    MOTOR_CHASSIS_LEFT = 0,
    MOTOR_CHASSIS_RIGHT = 1,
    MOTOR_GIMBAL_1 = 2,
    MOTOR_GIMBAL_2 = 3,
    MOTOR_COUNT = 4
} MotorId;

/* 兼容旧代码：旧工程只认左右两个电机名。 */
#define MOTOR_LEFT              MOTOR_CHASSIS_LEFT
#define MOTOR_RIGHT             MOTOR_CHASSIS_RIGHT

typedef enum {
    MOTOR_COAST = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_BRAKE
} MotorDir;

/*
 * Motor_Init：初始化四个步进驱动器的 STEP/DIR 管脚和内部调度状态。
 * 不要用于：还想保留 TB6612 PWM 的旧工程路径。
 */
void Motor_Init(void);

/*
 * Motor_Task：根据当前命令补发 STEP 脉冲。
 * 使用场景：App_Task 周期调用，保持步进运动持续进行。
 */
void Motor_Task(void);

/*
 * Motor_Set：设置单个电机的方向和速度命令。
 * 速度命令沿用旧工程 0~4000 的量程，实际会被换算成 STEP 发脉冲节奏。
 */
void Motor_Set(MotorId motor, MotorDir dir, uint16_t command);

/*
 * Motor_SetSpeed：继续保留旧的左右轮接口，便于 motion/speed_control 复用。
 * 现在它只驱动底盘左右两个步进电机。
 */
void Motor_SetSpeed(int16_t left, int16_t right);

/*
 * Motor_SetAllStop：停止四个步进电机并清掉积累的步进调度量。
 * 使用场景：状态机切换、测试结束、异常停车。
 */
void Motor_SetAllStop(void);

/*
 * Motor_Stop：兼容旧接口，等价于 Motor_SetAllStop。
 */
void Motor_Stop(void);

/*
 * Motor_GetCommand：读取某个电机当前命令。
 * 使用场景：测试页/调试页观察当前控制量。
 */
int16_t Motor_GetCommand(MotorId motor);

/*
 * Motor_GetStepCount：读取某个电机自上次清零后的累计 STEP 数。
 * 使用场景：测试时观察电机是否真的在走。
 */
int32_t Motor_GetStepCount(MotorId motor);

#endif
