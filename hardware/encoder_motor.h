#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H

#include <stdint.h>

#define ENCODER_MOTOR_COUNT       (2U)
#define ENCODER_MOTOR_LEFT        (0U)
#define ENCODER_MOTOR_RIGHT       (1U)

typedef enum {
    ENCODER_MOTOR_MODE_NORMAL = 0,
    ENCODER_MOTOR_MODE_CALIBRATION_OPEN_LOOP,
    ENCODER_MOTOR_MODE_CALIBRATION_CLOSED_LOOP
} EncoderMotorMode;

typedef struct {
    int16_t startPwm;
    int16_t runStartPwm;
    int16_t integralLimitPwm;
    int32_t ffQ1024;
    int32_t kpQ1024;
    int32_t kiQ1024;
} EncoderMotorTuning;

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

void EncoderMotor_Init(void);
void EncoderMotor_SetTargets(int16_t leftCps, int16_t rightCps);
/* 正常闭环目标，单位与 Task5 target/move 相同：encoder count/控制周期。 */
void EncoderMotor_SetPeriodTargets(int16_t leftCounts,
    int16_t rightCounts);
/* 仅在目标为0时允许按编码反馈输出反向阻尼PWM。 */
void EncoderMotor_SetZeroTargetBrake(uint8_t motorIndex, uint8_t enabled);
void EncoderMotor_SetTarget(uint8_t motorIndex, int16_t targetCps);
void EncoderMotor_RunControlPeriod(void);
void EncoderMotor_Stop(void);
void EncoderMotor_HandleGPIOInterrupt(void);

/* Task5 calibration interfaces. PWM values are raw compare counts. */
void EncoderMotor_EnterCalibration(void);
void EncoderMotor_ExitCalibration(void);
void EncoderMotor_SetOpenLoopPwm(int16_t leftPwm, int16_t rightPwm);
void EncoderMotor_SetCalibrationTargets(int16_t leftCounts,
    int16_t rightCounts);
void EncoderMotor_SetTuning(const EncoderMotorTuning *left,
    const EncoderMotorTuning *right);
void EncoderMotor_ClearIntegral(void);
void EncoderMotor_GetSnapshot(EncoderMotorSnapshot *snapshot);

int16_t EncoderMotor_GetTarget(uint8_t motorIndex);
int16_t EncoderMotor_GetPwm(uint8_t motorIndex);
int32_t EncoderMotor_GetTotalCount(uint8_t motorIndex);
void EncoderMotor_ResetTotalCount(uint8_t motorIndex);
void EncoderMotor_ResetAllCounts(void);

#endif
