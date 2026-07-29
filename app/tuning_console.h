#ifndef TUNING_CONSOLE_H
#define TUNING_CONSOLE_H

#include <stdint.h>

/* Task5两点前馈标定的内部阶段。 */
typedef enum {
    TUNING_CONSOLE_SET_STAGE_IDLE = 0, /* 没有正在执行的set采样。 */
    TUNING_CONSOLE_SET_STAGE_SETTLING, /* 已给速度，等待电机进入稳态。 */
    TUNING_CONSOLE_SET_STAGE_SAMPLING  /* 正在累计编码器平均值。 */
} TuningConsoleSetStage;

/* 左右轮两点标定各自的结果状态。 */
typedef enum {
    TUNING_CONSOLE_SET_RESULT_NONE = 0,
    TUNING_CONSOLE_SET_RESULT_RUNNING,
    TUNING_CONSOLE_SET_RESULT_POINT_STORED,
    TUNING_CONSOLE_SET_RESULT_FF_APPLIED,
    TUNING_CONSOLE_SET_RESULT_ERROR
} TuningConsoleSetResult;

/* Task5 H7 LCD页面编号，只决定显示内容，不改变控制模式。 */
typedef enum {
    TUNING_CONSOLE_OLED_FF = 0,
    TUNING_CONSOLE_OLED_START,
    TUNING_CONSOLE_OLED_SPEED,
    TUNING_CONSOLE_OLED_PID,
    TUNING_CONSOLE_OLED_GRAY,
    TUNING_CONSOLE_OLED_GIMBAL
} TuningConsoleOledPage;

/*
 * Task5显示只读快照：集中保存底盘标定、灰度、视觉和姿态页需要的数据。
 * 数值来自多个控制模块，调用方只能显示，禁止写回作为控制命令。
 */
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
    uint16_t visionYawGainQ1024;
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
    uint8_t h7ImuFresh;
    uint32_t h7ImuAgeMs;
    int16_t h7RollX100;
    int16_t h7PitchX100;
    int32_t h7YawX100;
    int32_t h7YawRateX100PerSec;
} TuningConsoleDisplayStatus;

/* 进入底盘调参会话并默认显示FF页；不会自动让电机转动。 */
void TuningConsole_Start(void);

/* GMR Task5入口：显示M0航向页；首帧有效yaw到达后会启动前进保持。 */
void TuningConsole_StartYaw(void);

/* 退出Task5并停止底盘、视觉和姿态实验输出。 */
void TuningConsole_Stop(void);

/* Task5启用时由5ms Comm任务调用，解析UART命令并发送低频状态。 */
void TuningConsole_Task(void);

/* Task5底盘测试的10ms控制入口；只能由CarControl任务调用。 */
void TuningConsole_ChassisControlPeriod(void);

/* 当前处于Task5调参会话时返回1。 */
uint8_t TuningConsole_IsActive(void);

/* 原子读取Task5显示数据；只读，不触发采样或电机命令。 */
void TuningConsole_GetDisplayStatus(TuningConsoleDisplayStatus *status);

#endif
