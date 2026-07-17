#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/*
 * MotorId：底盘两路编码电机和云台两路步进电机的统一逻辑编号。
 * MOTOR_GIMBAL_1 为左右轴，MOTOR_GIMBAL_2 为上下轴。
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
 * Motor_Init：初始化编码底盘、TIMA0 PWM和两路云台STEP调度。
 */
void Motor_Init(void);

/*
 * Motor_Task：电机主循环任务。
 * 使用场景：App_Task 周期调用。
 * 说明：ccs1.2 起 STEP 脉冲由 TIMG0 中断调度，本函数暂时不阻塞、不发脉冲。
 */
void Motor_Task(void);

/* Motor_RunChassisControl：按CHASSIS_CONTROL_PERIOD_MS执行一次底盘速度PI。 */
void Motor_RunChassisControl(void);

/*
 * Motor_Set：设置单个电机的方向和速度命令。
 * 底盘命令单位为编码器count/s；云台命令单位仍为step/s。
 */
void Motor_Set(MotorId motor, MotorDir dir, uint16_t speedSps);

/* Motor_SetRampStep：单独设置某一路电机的加减速斜坡步长。 */
void Motor_SetRampStep(MotorId motor, uint16_t accelStepSps,
    uint16_t decelStepSps);

/* Motor_ResetRampStep：恢复某一路电机的默认全局斜坡。 */
void Motor_ResetRampStep(MotorId motor);

/*
 * Motor_SetChassisCommand：设置底盘左右编码电机目标count/s。
 * 使用场景：循迹和路线统一从这里驱动底盘。
 */
void Motor_SetChassisCommand(int16_t leftCps, int16_t rightCps);

/* 设置底盘正常闭环目标，单位为 encoder count/控制周期。 */
void Motor_SetChassisPeriodCommand(int16_t leftCounts,
    int16_t rightCounts);

/*
 * Motor_SetAllStop：底盘PWM清零，并停止两路云台STEP输出。
 * 使用场景：状态机切换、测试结束、异常停车。
 */
void Motor_SetAllStop(void);

/* Motor_Stop：停止两路编码底盘和两路云台步进电机。 */
void Motor_Stop(void);

/*
 * Motor_GetCommand：读取当前有符号目标；底盘为count/s，云台为step/s。
 * 使用场景：测试页/调试页观察当前控制量。
 */
int16_t Motor_GetCommand(MotorId motor);

/*
 * Motor_GetStepCount：底盘返回累计编码器count，云台返回累计STEP。
 */
int32_t Motor_GetStepCount(MotorId motor);

/* Motor_ResetStepCount：底盘清编码器count，云台清STEP计数。 */
void Motor_ResetStepCount(MotorId motor);

/* Motor_ResetAllStepCounts：清零底盘编码器count和云台STEP计数。 */
void Motor_ResetAllStepCounts(void);

#endif
