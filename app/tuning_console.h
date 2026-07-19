#ifndef TUNING_CONSOLE_H
#define TUNING_CONSOLE_H

#include <stdint.h>

typedef enum {
    TUNING_CONSOLE_SET_STAGE_IDLE = 0,
    TUNING_CONSOLE_SET_STAGE_SETTLING,
    TUNING_CONSOLE_SET_STAGE_SAMPLING
} TuningConsoleSetStage;

typedef enum {
    TUNING_CONSOLE_SET_RESULT_NONE = 0,
    TUNING_CONSOLE_SET_RESULT_RUNNING,
    TUNING_CONSOLE_SET_RESULT_POINT_STORED,
    TUNING_CONSOLE_SET_RESULT_FF_APPLIED,
    TUNING_CONSOLE_SET_RESULT_ERROR
} TuningConsoleSetResult;

typedef enum {
    TUNING_CONSOLE_OLED_FF = 0,
    TUNING_CONSOLE_OLED_START,
    TUNING_CONSOLE_OLED_SPEED,
    TUNING_CONSOLE_OLED_PID,
    TUNING_CONSOLE_OLED_GRAY,
    TUNING_CONSOLE_OLED_GIMBAL
} TuningConsoleOledPage;

typedef struct {
    TuningConsoleOledPage oledPage;
    TuningConsoleSetStage setStage;
    TuningConsoleSetResult leftResult;
    TuningConsoleSetResult rightResult;
    uint32_t sampleCount;
    uint32_t sampleTarget;
    int32_t leftFfQ1024;
    int32_t rightFfQ1024;
    int32_t leftStartPercent;
    int32_t rightStartPercent;
    int32_t leftRunStartPercent;
    int32_t rightRunStartPercent;
    int32_t leftPwmPercent;
    int32_t rightPwmPercent;
    int32_t leftTargetCounts;
    int32_t rightTargetCounts;
    int32_t leftFeedbackCounts;
    int32_t rightFeedbackCounts;
    int32_t leftAverageCounts;
    int32_t rightAverageCounts;
    uint8_t grayMask;
    uint8_t gimbalMode;
    uint8_t visionMode;
    uint8_t visionHasFrame;
    uint32_t visionFrameCount;
    int16_t visionRawX;
    int16_t visionRawY;
    int16_t visionStageScaleX10;
    int16_t visionCommandX;
    int16_t visionCommandY;
    uint8_t visionYawBoostActive;
    uint8_t gimbalState;
    uint8_t gimbalHoldEnabled;
    uint8_t gimbalFeedForwardEnabled;
    uint8_t gimbalFeedbackFresh;
    uint8_t gimbalFeedForwardFresh;
    uint16_t gimbalCalibrationCount;
    uint16_t gimbalCalibrationTarget;
    int32_t gimbalYawX100;
    int32_t gimbalRateX100PerSec;
    int32_t gimbalAngleErrorX100;
    int16_t gimbalCommandSps;
} TuningConsoleDisplayStatus;

/* Start and stop the Task5 UART calibration session. */
void TuningConsole_Start(void);
void TuningConsole_Stop(void);

/* Run from the 5 ms communication task while Task5 is selected. */
void TuningConsole_Task(void);
void TuningConsole_ChassisControlPeriod(void);
uint8_t TuningConsole_IsActive(void);

/* Read the current set progress and FF results for the Task5 OLED page. */
void TuningConsole_GetDisplayStatus(TuningConsoleDisplayStatus *status);

#endif
