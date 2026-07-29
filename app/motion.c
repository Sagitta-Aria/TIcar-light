/*
 * 运动语义薄封装：向底层Motor模块提交底盘速度，或统一停止底盘与云台。
 * 供状态机和循迹控制器使用；这里不做编码器PI，也不负责直角检测。
 */
#include "motion.h"

#include "motor.h"

/*
 * 作用：把无符号底盘目标count/s压到int16_t可表达范围。
 * 使用场景：Motion_Forward/Backward/Turn* 生成左右轮有符号目标。
 */
static int16_t Motion_ToSignedSpeed(uint16_t speedCps)
{
    return (speedCps > 32767U) ? 32767 : (int16_t)speedCps;
}

/*
 * 作用：停止所有电机速度输出。
 * 使用场景：状态机退出运动状态、错误状态或人工返回菜单时。
 * 说明：这里调用 Motor_Stop，会影响底盘和云台，云台单独停止请用 gimbal 模块接口。
 */
void Motion_Stop(void)
{
    Motor_Stop();
}

/*
 * 作用：设置底盘左右轮的有符号目标count/s。
 * 使用场景：循迹、路线外环或手写运动动作需要直接控制底盘时。
 */
void Motion_SetChassisCommand(int16_t leftSpeedCps, int16_t rightSpeedCps)
{
    Motor_SetChassisCommand(leftSpeedCps, rightSpeedCps);
}

void Motion_SetChassisPeriodCommand(int16_t leftCounts, int16_t rightCounts)
{
    Motor_SetChassisPeriodCommand(leftCounts, rightCounts);
}

/* 作用：让底盘以相同目标count/s前进。 */
void Motion_Forward(uint16_t speedCps)
{
    int16_t speed = Motion_ToSignedSpeed(speedCps);
    Motion_SetChassisCommand(speed, speed);
}

/* 作用：让底盘以相同目标count/s后退。 */
void Motion_Backward(uint16_t speedCps)
{
    int16_t speed = Motion_ToSignedSpeed(speedCps);
    Motion_SetChassisCommand((int16_t)-speed, (int16_t)-speed);
}

/* 作用：让底盘原地左转，主要用于测试或后续任务动作。 */
void Motion_TurnLeft(uint16_t speedCps)
{
    int16_t speed = Motion_ToSignedSpeed(speedCps);
    Motion_SetChassisCommand((int16_t)-speed, speed);
}

/* 作用：让底盘原地右转，主要用于测试或后续任务动作。 */
void Motion_TurnRight(uint16_t speedCps)
{
    int16_t speed = Motion_ToSignedSpeed(speedCps);
    Motion_SetChassisCommand(speed, (int16_t)-speed);
}
