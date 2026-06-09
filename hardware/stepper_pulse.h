#ifndef STEPPER_PULSE_H
#define STEPPER_PULSE_H

#include <stdint.h>

#include "motor.h"

/*
 * StepperPulse_Init：启动 STEP 定时器调度器。
 * 使用场景：Board 完成 TIMG0 配置后，由 Motor_Init 调用。
 * 说明：本模块只产生 STEP 脉冲，DIR 方向仍由 motor.c 管理。
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
 * StepperPulse_StopAll：停止四路 STEP 输出并清空调度累加器。
 * 使用场景：状态切换、异常停车和电机初始化。
 */
void StepperPulse_StopAll(void);

/*
 * StepperPulse_GetStepCount：读取累计 STEP 数。
 * 使用场景：路线测距、测试页/日志观察电机是否收到脉冲。
 * 说明：返回值带方向符号，正负由 DIR 命令决定。
 */
int32_t StepperPulse_GetStepCount(MotorId motor);

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
 * StepperPulse_HandleTimerInterrupt：TIMG0 中断入口分发函数。
 * 使用场景：system/interrupt.c 的 TIMG0_IRQHandler 调用。
 * 说明：中断里不打印、不刷屏、不等待，只做 GPIO 翻转。
 */
void StepperPulse_HandleTimerInterrupt(void);

#endif
