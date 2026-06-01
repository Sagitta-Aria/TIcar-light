#ifndef STEPPER_PIN_TEST_H
#define STEPPER_PIN_TEST_H

#include <stdint.h>

/*
 * StepperPinTest_InitPins：初始化云台 STEP/DIR 测试引脚。
 * 使用场景：CAR_GIMBAL_PIN_TEST_BUILD=1 时，由 Board_Init 调用。
 * 不使用场景：正常小车固件不要调用，正常固件由 generated GPIO 初始化接管。
 */
void StepperPinTest_InitPins(void);

/*
 * StepperPinTest_Start：从左右轴正向测试开始。
 * 使用场景：CAR_GIMBAL_PIN_TEST_BUILD=1 时，由 App_Init 调用。
 */
void StepperPinTest_Start(void);

/*
 * StepperPinTest_Task：直接 bit-bang 输出 STEP 脉冲。
 * 使用场景：CAR_GIMBAL_PIN_TEST_BUILD=1 时，由 App_Task 周期调用。
 * 说明：不依赖 TIMG0，不读电机反馈；电机没接也不会阻塞。
 */
void StepperPinTest_Task(void);

#endif
