#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H

#include <stdint.h>

#define ENCODER_MOTOR_COUNT       (2U) /* 固定两路编码底盘电机。 */
#define ENCODER_MOTOR_LEFT        (0U) /* 数组索引0：左轮/TB6612 B通道。 */
#define ENCODER_MOTOR_RIGHT       (1U) /* 数组索引1：右轮/TB6612 A通道。 */

/* 底盘驱动当前运行模式；切换模式会清理不兼容的积分或PWM状态。 */
typedef enum {
    ENCODER_MOTOR_MODE_NORMAL = 0,             /* 比赛用左右轮独立速度闭环。 */
    ENCODER_MOTOR_MODE_CALIBRATION_OPEN_LOOP,  /* Task5直接输出raw PWM。 */
    ENCODER_MOTOR_MODE_CALIBRATION_CLOSED_LOOP,/* Task5速度闭环标定。 */
    ENCODER_MOTOR_MODE_CROSS_COUPLED_PWM       /* PWM输出加左右编码器同步修正。 */
} EncoderMotorMode;

/* 单轮速度PI/前馈参数；PWM字段为raw count，增益字段为Q1024。 */
typedef struct {
    int16_t startPwm;
    int16_t runStartPwm;
    int16_t integralLimitPwm;
    int32_t ffQ1024;
    int32_t kpQ1024;
    int32_t kiQ1024;
} EncoderMotorTuning;

/* Task5和菜单使用的原子诊断快照；读取不会清零编码器窗口。 */
typedef struct {
    EncoderMotorMode mode;
    uint32_t sampleSequence;
    int32_t targetCounts[ENCODER_MOTOR_COUNT];
    int32_t feedbackCounts[ENCODER_MOTOR_COUNT];
    int16_t outputPwm[ENCODER_MOTOR_COUNT];
    uint32_t pwmCompareCounts[ENCODER_MOTOR_COUNT];
    int32_t integralOutputPwm[ENCODER_MOTOR_COUNT];
    uint8_t startupActive[ENCODER_MOTOR_COUNT];
    EncoderMotorTuning tuning[ENCODER_MOTOR_COUNT];
    uint32_t rightEncoderAInterruptCount;
    uint32_t rightEncoderBInterruptCount;
    uint8_t rightEncoderALevel;
    uint8_t rightEncoderBLevel;
} EncoderMotorSnapshot;

/* 初始化TIMA0 PWM、方向GPIO、编码器GPIO中断和默认速度环参数。 */
void EncoderMotor_Init(void);

/* 兼容接口：设置左右目标count/s，内部换算到count/20ms。 */
void EncoderMotor_SetTargets(int16_t leftCps, int16_t rightCps);

/* 正常闭环目标，单位与Task5相同：固定encoder count/20ms速度刻度。 */
void EncoderMotor_SetPeriodTargets(int16_t leftCounts,
    int16_t rightCounts);

/* 直接PWM命令；底层按最新左右编码速度差做交叉同步修正。 */
void EncoderMotor_SetCrossCoupledPwm(int16_t leftPwm, int16_t rightPwm,
    int32_t syncGainQ1024, uint16_t syncLimitPwm);

/* 仅在目标为0时允许按编码反馈输出反向阻尼PWM。 */
void EncoderMotor_SetZeroTargetBrake(uint8_t motorIndex, uint8_t enabled);

/* 兼容单轮count/s目标接口；非法motorIndex会被忽略。 */
void EncoderMotor_SetTarget(uint8_t motorIndex, int16_t targetCps);

/* 每10ms读取并清零编码器速度窗口，然后计算一次PWM；禁止从ISR调用。 */
void EncoderMotor_RunControlPeriod(void);

/* 两轮PWM和目标清零，同时清除积分、起步和同步状态。 */
void EncoderMotor_Stop(void);

/* GPIOA/GPIOB中断分发入口；只更新正交编码器计数，不运行PI。 */
void EncoderMotor_HandleGPIOInterrupt(void);

/* 以下为Task5标定接口；PWM值均是raw timer count，不是百分比。 */
void EncoderMotor_EnterCalibration(void);
void EncoderMotor_ExitCalibration(void);
void EncoderMotor_SetOpenLoopPwm(int16_t leftPwm, int16_t rightPwm);
void EncoderMotor_SetCalibrationTargets(int16_t leftCounts,
    int16_t rightCounts);
void EncoderMotor_SetTuning(const EncoderMotorTuning *left,
    const EncoderMotorTuning *right);
void EncoderMotor_ClearIntegral(void);
void EncoderMotor_GetSnapshot(EncoderMotorSnapshot *snapshot);

/* 读取单轮当前目标、PWM或累计count；非法索引返回0。 */
int16_t EncoderMotor_GetTarget(uint8_t motorIndex);
int16_t EncoderMotor_GetPwm(uint8_t motorIndex);
int32_t EncoderMotor_GetTotalCount(uint8_t motorIndex);

/* 原子读取左右轮累计编码数，避免跨两个单轮读取产生时间偏差。 */
void EncoderMotor_GetTotalCounts(int32_t *leftCount, int32_t *rightCount);

/* 清零指定单轮累计编码器count；不会改变当前速度目标。 */
void EncoderMotor_ResetTotalCount(uint8_t motorIndex);

/* 原子清零左右累计count和当前速度采样窗口。 */
void EncoderMotor_ResetAllCounts(void);

#endif
