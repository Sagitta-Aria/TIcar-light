#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/*
 * MotorId：四个闭环步进驱动器的逻辑编号。
 * 底盘左右电机放在前两个编号，云台两个电机放在后两个编号。
 */
typedef enum {
    MOTOR_CHASSIS_LEFT = 0,
    MOTOR_CHASSIS_RIGHT = 1,
    MOTOR_GIMBAL_1 = 2,
    MOTOR_GIMBAL_2 = 3,
    MOTOR_COUNT = 4
} MotorId;

typedef enum {
    MOTOR_COAST = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_BRAKE
} MotorDir;

/*
 * Motor_Init：初始化四个步进驱动器的 STEP/DIR 管脚和内部调度状态。
 */
void Motor_Init(void);

/*
 * Motor_Task：电机主循环任务。
 * 使用场景：App_Task 周期调用。
 * 说明：ccs1.2 起 STEP 脉冲由 TIMG0 中断调度，本函数暂时不阻塞、不发脉冲。
 */
void Motor_Task(void);

/*
 * Motor_Set：设置单个电机的方向和速度命令。
 * 速度命令使用 0~CAR_MOTOR_COMMAND_MAX 的量程，实际会换算成 STEP 发脉冲节奏。
 */
void Motor_Set(MotorId motor, MotorDir dir, uint16_t command);

/*
 * Motor_SetChassisCommand：设置底盘左右两个步进电机的有符号速度命令。
 * 使用场景：循迹和路线统一从这里驱动底盘。
 */
void Motor_SetChassisCommand(int16_t leftCommand, int16_t rightCommand);

/*
 * Motor_SetAllStop：停止四个步进电机并清掉积累的步进调度量。
 * 使用场景：状态机切换、测试结束、异常停车。
 */
void Motor_SetAllStop(void);

/* Motor_Stop：停止四个步进电机。 */
void Motor_Stop(void);

/*
 * Motor_GetCommand：读取某个电机当前命令。
 * 使用场景：测试页/调试页观察当前控制量。
 */
int16_t Motor_GetCommand(MotorId motor);

/*
 * Motor_GetStepCount：读取某个电机累计 STEP 数。
 * 使用场景：路线测距、串口调试和实车标定。
 * 说明：返回值带方向符号，正负由 DIR 命令决定。
 */
int32_t Motor_GetStepCount(MotorId motor);

/* Motor_ResetStepCount：清零某个电机的 STEP 计数。 */
void Motor_ResetStepCount(MotorId motor);

/* Motor_ResetAllStepCounts：清零四个电机的 STEP 计数。 */
void Motor_ResetAllStepCounts(void);

#endif
