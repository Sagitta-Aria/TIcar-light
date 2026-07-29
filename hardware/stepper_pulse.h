#ifndef STEPPER_PULSE_H
#define STEPPER_PULSE_H

#include <stdint.h>

#include "motor.h"

/*
 * StepperPulse_Init：初始化 STEP 调度器，首个非零命令才启动定时器。
 * 使用场景：Board 完成 TIMG6 配置后，由 Motor_Init 调用。
 * 说明：TIMG6 有运动命令时以 20kHz 运行，两轴为零时停止中断。
 */
void StepperPulse_Init(void);

/*
 * StepperPulse_SetTarget：设置单个电机的 STEP 目标节奏。
 * 使用场景：Motor_Set 在方向脚稳定后调用。
 * 说明：speedSps 单位为 step/s，直接决定每秒输出多少个 STEP。
 */
void StepperPulse_SetTarget(MotorId motor, int8_t directionSign,
    uint16_t speedSps);

/*
 * 按给定方向输出精确数量的 STEP，上层的新速度命令会取消未完成的定步运动。
 * 仅用于云台轴；stepCount为0时等同于停止该轴。
 */
void StepperPulse_SetMoveTarget(MotorId motor, int8_t directionSign,
    uint16_t speedSps, uint32_t stepCount);

/* 定步运动尚有未输出脉冲或末个高电平尚未结束时返回1。 */
uint8_t StepperPulse_IsMoveActive(MotorId motor);

/*
 * StepperPulse_SetRampStep：单独设置某一路 STEP 斜坡速度。
 * 使用场景：Task4 强转时临时提高云台 yaw 轴响应，不影响底盘。
 */
void StepperPulse_SetRampStep(MotorId motor, uint16_t accelStepSps,
    uint16_t decelStepSps);

/*
 * StepperPulse_StopAll：停止两路云台 STEP 输出并清空调度累加器。
 * 使用场景：状态切换、异常停车和电机初始化。
 */
void StepperPulse_StopAll(void);

/*
 * StepperPulse_GetStepCount：读取累计 STEP 数。
 * 使用场景：路线测距、测试页/日志观察电机是否收到脉冲。
 * 说明：返回值带方向符号，正负由 DIR 命令决定。
 */
int32_t StepperPulse_GetStepCount(MotorId motor);

/* 读取斜坡后的当前有符号 STEP 输出频率；它不是机械轴编码器反馈。 */
int16_t StepperPulse_GetCurrentRate(MotorId motor);

/*
 * StepperPulse_ResetStepCount：清零单路 STEP 计数。
 * 使用场景：测试开始、路线重新标定起点。
 */
void StepperPulse_ResetStepCount(MotorId motor);

/*
 * StepperPulse_ResetAllStepCounts：清零全部 STEP 计数。
 * 使用场景：上电初始化或整车任务重新开始。
 */
void StepperPulse_ResetAllStepCounts(void);

/*
 * StepperPulse_HandleTimerInterrupt：TIMG6 中断入口分发函数。
 * 使用场景：system/interrupt.c 的 TIMG6_IRQHandler 调用。
 * 说明：中断里不打印、不刷屏、不等待，只做 GPIO 翻转。
 */
void StepperPulse_HandleTimerInterrupt(void);

#endif
