/*
 * Task5串口调参台：解析底盘FF/PI、灰度、视觉和双IMU姿态实验命令并输出SerialPlot帧。
 * Comm任务负责命令解析，CarControl任务调用专用10ms入口；两条路径通过受控状态共享数据。
 * 在线参数只保存在RAM，复位不会写Flash；退出Task5时必须停止所有测试输出。
 */
#include "library_config.h"

#if CAR_PROFILE_IS_FULL

#include "tuning_console.h"

#include "FreeRTOS.h"
#include "task.h"

#include "control_config.h"
#include "body_motion.h"
#include "encoder_motor.h"
#include "gimbal.h"
#include "gimbal_attitude.h"
#include "gray.h"
#include "log_uart.h"
#include "motor.h"
#include "motor_enable.h"
#include "motor_no_yaw.h"
#include "staticconfig.h"
#include "tuning_common.h"
#include "vision.h"

#define TUNING_LINE_MAX                 (80U)
#define TUNING_TOKEN_MAX                (5U)
#define TUNING_STATUS_PERIOD_MS         (500U)
#define TUNING_GIMBAL_PLOT_PERIOD_MS    (20U)
#define TUNING_VISION_PLOT_PERIOD_MS    (20U)
#define TUNING_TARGET_COUNT_LIMIT \
    ((int32_t)CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD)
#define TUNING_SET_SETTLE_MS            (4000U)
#define TUNING_SET_SAMPLE_DURATION_MS   (1000U)
#define TUNING_SET_SAMPLE_WINDOWS \
    (TUNING_SET_SAMPLE_DURATION_MS / CHASSIS_CONTROL_PERIOD_MS)
#define TUNING_PLOT_FIT_MARKER           (-32768L)
#define TUNING_ENCODER_SPEED_MAX_COUNTS \
    ((int32_t)CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD)
#define TUNING_ENCODER_DISTANCE_LIMIT    (1000000000L)

#if ((TUNING_SET_SAMPLE_DURATION_MS % CHASSIS_CONTROL_PERIOD_MS) != 0U)
#error "Task5 set sampling duration must contain whole chassis periods"
#endif

typedef enum {
    TUNING_SET_IDLE = 0,
    TUNING_SET_SETTLING,
    TUNING_SET_SAMPLING
} TuningSetState;

typedef enum {
    TUNING_MODE_CHASSIS = 0,
    TUNING_MODE_GIMBAL,
    TUNING_MODE_VISION
} TuningMode;

typedef struct {
    int16_t pwm;
    int64_t feedbackSum;
    uint32_t sampleCount;
    uint8_t valid;
} TuningSetPoint;

static char g_line[TUNING_LINE_MAX];
static uint8_t g_lineLength;
static uint8_t g_discardLine;
static uint8_t g_active;
static uint8_t g_forceStatus;
static uint8_t g_skipNextSample;
static TickType_t g_lastStatusTime;
static uint32_t g_lastSampleSequence;
static int64_t g_feedbackSum[ENCODER_MOTOR_COUNT];
static uint32_t g_averageSampleCount;
static TuningSetState g_setState;
static TuningConsoleOledPage g_oledPage;
static TickType_t g_setStateStartTime;
static int16_t g_setPwm[ENCODER_MOTOR_COUNT];
static TuningSetPoint g_setReference[ENCODER_MOTOR_COUNT];
static TuningConsoleSetResult g_setResult[ENCODER_MOTOR_COUNT];
static volatile uint8_t g_grayActive;
static volatile uint8_t g_grayMask;
static int16_t g_encoderMoveSpeed[ENCODER_MOTOR_COUNT];
static int32_t g_encoderMoveStart[ENCODER_MOTOR_COUNT];
static int32_t g_encoderMoveDistance[ENCODER_MOTOR_COUNT];
static uint8_t g_encoderMoveActive;
static TuningMode g_tuningMode;
static uint8_t g_gimbalPlotEnabled;
static TickType_t g_lastGimbalPlotTime;
static StaticConfigMode g_visionSource;
static uint8_t g_visionPlotEnabled;
static TickType_t g_lastVisionPlotTime;

static void TuningConsole_CancelSetSampling(void);
static void TuningConsole_CancelEncoderMove(void);
static void TuningConsole_StopGrayTest(void);
static void TuningConsole_ResetAverage(void);

static void TuningConsole_StopVisionActivity(void)
{
    g_visionPlotEnabled = 0U;
    Vision_Stop();
    Gimbal_SetEnabled(0U);
    Gimbal_SetYawFeedForward(0);
    Gimbal_SetYawAttitudeCompensation(0);
    MotorEnable_SetGimbal(0U);
}

/*
 * 启动Task5视觉调试并选择唯一point/circle配置。
 * 副作用：清空视觉接收状态、启用两个云台轴；不再接受人工距离档。
 */
static void TuningConsole_StartVisionActivity(void)
{
    StaticConfig_SetActiveMode(g_visionSource);
    Vision_Start();
    Gimbal_SetTarget(0, 0);
    Gimbal_SetEnabled(1U);
    MotorEnable_SetGimbal(1U);
}

static void TuningConsole_StopChassisActivity(void)
{
    TuningConsole_CancelSetSampling();
    TuningConsole_CancelEncoderMove();
    TuningConsole_StopGrayTest();
    EncoderMotor_SetOpenLoopPwm(0, 0);
    EncoderMotor_ExitCalibration();
    TuningConsole_ResetAverage();
}

static void TuningConsole_SetMode(TuningMode mode)
{
    if (mode == TUNING_MODE_GIMBAL) {
#if !CAR_LIBRARY_GIMBAL_ATTITUDE_ENABLED
        LogUart_SendString(
            "#ERR dual-IMU gimbal library is disabled in library_config.h\r\n");
        return;
#endif
        TuningConsole_StopVisionActivity();
        TuningConsole_StopChassisActivity();
        GimbalAttitude_Start();
        MotorEnable_SetGimbal(1U);
        g_tuningMode = TUNING_MODE_GIMBAL;
        g_gimbalPlotEnabled = 1U;
        g_lastGimbalPlotTime = xTaskGetTickCount();
        g_oledPage = TUNING_CONSOLE_OLED_GIMBAL;
        LogUart_SendString(
            "#OK mode=gimbal; H7 feedback active, JY61 feedforward off\r\n");
    } else if (mode == TUNING_MODE_VISION) {
#if !CAR_LIBRARY_GIMBAL_TRACKING_ENABLED
        LogUart_SendString(
            "#ERR 2D vision gimbal library is disabled in library_config.h\r\n");
        return;
#endif
        TuningConsole_StopChassisActivity();
        GimbalAttitude_Stop();
        TuningConsole_StopVisionActivity();
        TuningConsole_StartVisionActivity();
        g_tuningMode = TUNING_MODE_VISION;
        g_gimbalPlotEnabled = 0U;
        g_visionPlotEnabled = 1U;
        g_lastVisionPlotTime = xTaskGetTickCount();
        LogUart_SendString(
            "#OK mode=vision; visual tracking and B plot enabled\r\n");
    } else {
        TuningConsole_StopVisionActivity();
        GimbalAttitude_Stop();
        MotorEnable_SetGimbal(0U);
        EncoderMotor_EnterCalibration();
        g_tuningMode = TUNING_MODE_CHASSIS;
        g_gimbalPlotEnabled = 0U;
        g_oledPage = TUNING_CONSOLE_OLED_FF;
        TuningConsole_ResetAverage();
        LogUart_SendString("#OK mode=chassis; gimbal stopped\r\n");
    }
    g_forceStatus = 1U;
}

#define TuningConsole_TextEquals   TuningCommon_TextEquals
#define TuningConsole_ToLower      TuningCommon_ToLower
#define TuningConsole_ParseInt32   TuningCommon_ParseInt32
#define TuningConsole_Split        TuningCommon_Split
#define TuningConsole_PercentToPwm TuningCommon_PercentToPwm
#define TuningConsole_PwmToPercent TuningCommon_PwmToPercent
#define TUNING_PWM_PERCENT_MAX     TUNING_COMMON_PWM_PERCENT_MAX

static uint8_t TuningConsole_ParsePair(char *tokens[], int32_t *left,
    int32_t *right)
{
    if ((TuningConsole_ParseInt32(tokens[1], left) == 0U) ||
        (TuningConsole_ParseInt32(tokens[2], right) == 0U)) {
        LogUart_SendString("#ERR expected two integers\r\n");
        return 0U;
    }
    return 1U;
}

static void TuningConsole_SendPair(const char *name, int32_t left,
    int32_t right)
{
    LogUart_SendString("# ");
    LogUart_SendString(name);
    LogUart_SendString("=");
    LogUart_SendSigned(left);
    LogUart_SendString(",");
    LogUart_SendSigned(right);
    LogUart_SendString("\r\n");
}

static void TuningConsole_SendUnsignedPair(const char *name, uint32_t left,
    uint32_t right)
{
    LogUart_SendString("# ");
    LogUart_SendString(name);
    LogUart_SendString("=");
    LogUart_SendUnsigned(left);
    LogUart_SendString(",");
    LogUart_SendUnsigned(right);
    LogUart_SendString("\r\n");
}

static const char *TuningConsole_GetModeName(EncoderMotorMode mode)
{
    if (mode == ENCODER_MOTOR_MODE_CALIBRATION_OPEN_LOOP) {
        return "open";
    }
    if (mode == ENCODER_MOTOR_MODE_CALIBRATION_CLOSED_LOOP) {
        return "closed";
    }
    return "normal";
}

static const char *TuningConsole_GetSetStateName(void)
{
    if (g_setState == TUNING_SET_SETTLING) {
        return "settle";
    }
    if (g_setState == TUNING_SET_SAMPLING) {
        return "sample";
    }
    return "idle";
}

static int32_t TuningConsole_GetAverage(uint8_t motorIndex)
{
    if ((motorIndex >= ENCODER_MOTOR_COUNT) ||
        (g_averageSampleCount == 0U)) {
        return 0;
    }
    return (int32_t)(g_feedbackSum[motorIndex] /
        (int64_t)g_averageSampleCount);
}

static void TuningConsole_ResetAverage(void)
{
    EncoderMotorSnapshot snapshot;

    EncoderMotor_GetSnapshot(&snapshot);
    g_feedbackSum[ENCODER_MOTOR_LEFT] = 0;
    g_feedbackSum[ENCODER_MOTOR_RIGHT] = 0;
    g_averageSampleCount = 0U;
    g_lastSampleSequence = snapshot.sampleSequence;
    g_skipNextSample = 1U;
}

static void TuningConsole_StopGrayTest(void)
{
    g_grayActive = 0U;
    g_grayMask = 0U;
    EncoderMotor_EnterCalibration();
}

static void TuningConsole_StartGrayTest(void)
{
    TuningConsole_CancelSetSampling();
    TuningConsole_CancelEncoderMove();
    EncoderMotor_ExitCalibration();
    g_grayMask = Gray_ReadDigitalMaskFast();
    g_grayActive = 1U;
    g_oledPage = TUNING_CONSOLE_OLED_GRAY;
    TuningConsole_ResetAverage();
    LogUart_SendString("#OK gray on; Task1 line control active\r\n");
    g_forceStatus = 1U;
}

static void TuningConsole_UpdateAverage(void)
{
    EncoderMotorSnapshot snapshot;

    EncoderMotor_GetSnapshot(&snapshot);
    if (snapshot.sampleSequence == g_lastSampleSequence) {
        return;
    }
    g_lastSampleSequence = snapshot.sampleSequence;
    if (g_skipNextSample != 0U) {
        g_skipNextSample = 0U;
        return;
    }
    g_feedbackSum[ENCODER_MOTOR_LEFT] +=
        snapshot.feedbackCounts[ENCODER_MOTOR_LEFT];
    g_feedbackSum[ENCODER_MOTOR_RIGHT] +=
        snapshot.feedbackCounts[ENCODER_MOTOR_RIGHT];
    ++g_averageSampleCount;
}

static void TuningConsole_SendHelp(void)
{
    LogUart_SendString(
        "# mode chassis|gimbal|vision - select Task5 owner\r\n");
    LogUart_SendString(
        "# gcal | ghold on|off | gff on|off | gplot on|off | gshow\r\n");
    LogUart_SendString(
        "# gsteps|gkff|gkp|grkp|glpf|gbeta|gpred VALUE\r\n");
    LogUart_SendString(
        "# gsign|gh7sign|gjysign -1|1 | gmax|gaccel|glimit VALUE\r\n");
    LogUart_SendString(
        "# vconfig center|circle | vplot on|off | vshow\r\n");
    LogUart_SendString("# Task5 PWM commands use percent:\r\n");
    LogUart_SendString("# set L R       -100..100%, sample FF point\r\n");
    LogUart_SendString("# set clear     clear saved FF points\r\n");
    LogUart_SendString("# pwm L R       open loop -100..100%\r\n");
    LogUart_SendString("# target L R    closed-loop count/20ms\r\n");
    LogUart_SendString(
        "# move LS RS LD RD speed 1..100, signed distance count\r\n");
    LogUart_SendString("# start L R     low-speed PWM 0..100%\r\n");
    LogUart_SendString("# runstart L R  running PWM 0..100%\r\n");
    LogUart_SendString("# ff|kp|ki L R  Q1024 values\r\n");
    LogUart_SendString("# ilim L R      integral limit 0..100%\r\n");
    LogUart_SendString("# ffcalc PCT1 C1 PCT2 C2\r\n");
    LogUart_SendString("# gray [on|off] Task1 gray/motor test\r\n");
    LogUart_SendString("# oled ff|start|speed|pid|gray|gimbal\r\n");
    LogUart_SendString("# avg | clear | stop | show | help\r\n");
}

static void TuningConsole_SelectOledPage(const char *name)
{
    if (TuningConsole_TextEquals(name, "ff") != 0U) {
        g_oledPage = TUNING_CONSOLE_OLED_FF;
    } else if (TuningConsole_TextEquals(name, "start") != 0U) {
        g_oledPage = TUNING_CONSOLE_OLED_START;
    } else if (TuningConsole_TextEquals(name, "speed") != 0U) {
        g_oledPage = TUNING_CONSOLE_OLED_SPEED;
    } else if (TuningConsole_TextEquals(name, "pid") != 0U) {
        g_oledPage = TUNING_CONSOLE_OLED_PID;
    } else if (TuningConsole_TextEquals(name, "gray") != 0U) {
        g_oledPage = TUNING_CONSOLE_OLED_GRAY;
    } else if (TuningConsole_TextEquals(name, "gimbal") != 0U) {
        g_oledPage = TUNING_CONSOLE_OLED_GIMBAL;
    } else {
        LogUart_SendString(
            "#ERR oled expects ff|start|speed|pid|gray|gimbal\r\n");
        return;
    }
    LogUart_SendString("#OK oled=");
    LogUart_SendString(name);
    LogUart_SendString("\r\n");
    g_forceStatus = 1U;
}

/* SerialPlot: wheel data, raw PWM A CC1, then persistent left/right FF. */
static void TuningConsole_SendPlotFrame(
    const EncoderMotorSnapshot *snapshot)
{
    LogUart_SendString("P ");
    LogUart_SendSigned(snapshot->targetCounts[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot->feedbackCounts[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendSigned(TuningConsole_PwmToPercent(
        snapshot->outputPwm[ENCODER_MOTOR_LEFT]));
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot->targetCounts[ENCODER_MOTOR_RIGHT]);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot->feedbackCounts[ENCODER_MOTOR_RIGHT]);
    LogUart_SendString(",");
    LogUart_SendSigned(TuningConsole_PwmToPercent(
        snapshot->outputPwm[ENCODER_MOTOR_RIGHT]));
    LogUart_SendString(",");
    LogUart_SendUnsigned(
        snapshot->pwmCompareCounts[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot->tuning[ENCODER_MOTOR_LEFT].ffQ1024);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot->tuning[ENCODER_MOTOR_RIGHT].ffQ1024);
    LogUart_SendString("\r\n");
}

/*
 * SerialPlot keeps only P-prefixed lines and expects exactly nine values.
 * Labels use its Arduino-style "name:value" syntax and are stripped safely.
 */
static void TuningConsole_SendPlotFitFrame(uint8_t motorNumber,
    int32_t pwm1Raw, int32_t count1Value, int32_t pwm2Raw,
    int32_t count2Value, int32_t ffQ1024, int32_t leftFfQ1024,
    int32_t rightFfQ1024)
{
    LogUart_SendString("P FIT:");
    LogUart_SendSigned(TUNING_PLOT_FIT_MARKER);
    LogUart_SendString(",motor:");
    LogUart_SendUnsigned(motorNumber);
    LogUart_SendString(",pwm1_raw:");
    LogUart_SendSigned(pwm1Raw);
    LogUart_SendString((motorNumber == 0U) ? ",count1:" :
        ",count1_x1000:");
    LogUart_SendSigned(count1Value);
    LogUart_SendString(",pwm2_raw:");
    LogUart_SendSigned(pwm2Raw);
    LogUart_SendString((motorNumber == 0U) ? ",count2:" :
        ",count2_x1000:");
    LogUart_SendSigned(count2Value);
    LogUart_SendString(",ff_q1024:");
    LogUart_SendSigned(ffQ1024);
    LogUart_SendString(",left_ff:");
    LogUart_SendSigned(leftFfQ1024);
    LogUart_SendString(",right_ff:");
    LogUart_SendSigned(rightFfQ1024);
    LogUart_SendString("\r\n");
}

static void TuningConsole_SendStatus(void)
{
    EncoderMotorSnapshot snapshot;

    if (g_tuningMode == TUNING_MODE_GIMBAL) {
        GimbalAttitudeSnapshot gimbal;

        GimbalAttitude_GetSnapshot(&gimbal);
        LogUart_SendString("G h7=");
        LogUart_SendUnsigned(gimbal.feedbackFresh);
        LogUart_SendString(" h7_age=");
        LogUart_SendUnsigned(gimbal.feedbackAngleAgeMs);
        LogUart_SendString("/");
        LogUart_SendUnsigned(gimbal.feedbackGyroAgeMs);
        LogUart_SendString(" jy_state=");
        LogUart_SendUnsigned((uint32_t)gimbal.motion.state);
        LogUart_SendString(" jy_cal=");
        LogUart_SendUnsigned(gimbal.motion.calibrationCount);
        LogUart_SendString("/");
        LogUart_SendUnsigned(gimbal.motion.calibrationTarget);
        LogUart_SendString(" hold=");
        LogUart_SendUnsigned(gimbal.holdEnabled);
        LogUart_SendString(" ff_gate=");
        LogUart_SendUnsigned(gimbal.feedForwardEnabled);
        LogUart_SendString(" ff_ready=");
        LogUart_SendUnsigned(gimbal.feedForwardFresh);
        LogUart_SendString(" h7_yaw_x100=");
        LogUart_SendSigned(gimbal.feedbackYawX100);
        LogUart_SendString(" h7_rate_x100_s=");
        LogUart_SendSigned(gimbal.feedbackRateX100PerSec);
        LogUart_SendString(" jy_rate_x100_s=");
        LogUart_SendSigned(gimbal.motion.yawRateFilteredX100PerSec);
        LogUart_SendString(" jy_bias_x100_s=");
        LogUart_SendSigned(gimbal.motion.gyroBiasX100PerSec);
        LogUart_SendString(" ref_yaw_x100=");
        LogUart_SendSigned(gimbal.referenceYawX100);
        LogUart_SendString(" angle_err_x100=");
        LogUart_SendSigned(gimbal.angleErrorX100);
        LogUart_SendString(" rate_ref_x100_s=");
        LogUart_SendSigned(gimbal.rateReferenceX100PerSec);
        LogUart_SendString(" rate_err_x100_s=");
        LogUart_SendSigned(gimbal.rateErrorX100PerSec);
        LogUart_SendString(" step=");
        LogUart_SendSigned(gimbal.currentStep);
        LogUart_SendString(" ff_sps=");
        LogUart_SendSigned(gimbal.feedForwardSps);
        LogUart_SendString(" rate_fb_sps=");
        LogUart_SendSigned(gimbal.rateFeedbackSps);
        LogUart_SendString(" cmd_sps=");
        LogUart_SendSigned(gimbal.commandSps);
        LogUart_SendString(" step_sps=");
        LogUart_SendSigned(gimbal.stepOutputSps);
        LogUart_SendString(" plot=");
        LogUart_SendUnsigned(g_gimbalPlotEnabled);
        LogUart_SendString("\r\n");
        return;
    }
    if (g_tuningMode == TUNING_MODE_VISION) {
        const StaticConfigGimbalTask *config =
            StaticConfig_GetActiveGimbal();

        LogUart_SendString("V running=");
        LogUart_SendUnsigned(Vision_IsRunning());
        LogUart_SendString(" frame=");
        LogUart_SendUnsigned(Vision_GetFrameCount());
        LogUart_SendString(" bad=");
        LogUart_SendUnsigned(Vision_GetBadFrameCount());
        LogUart_SendString(" config=");
        LogUart_SendString(config->name);
        LogUart_SendString(" raw=");
        LogUart_SendSigned(Vision_GetRawX());
        LogUart_SendString(",");
        LogUart_SendSigned(Vision_GetRawY());
        LogUart_SendString(" stage_x10=");
        LogUart_SendSigned(Vision_GetStageScaleX10());
        LogUart_SendString(" yaw_k_q1024=");
        LogUart_SendUnsigned(Vision_GetYawGainQ1024());
        LogUart_SendString(" error=");
        LogUart_SendSigned(Gimbal_GetErrorX());
        LogUart_SendString(",");
        LogUart_SendSigned(Gimbal_GetErrorY());
        LogUart_SendString(" cmd_sps=");
        LogUart_SendSigned(Gimbal_GetCommandX());
        LogUart_SendString(",");
        LogUart_SendSigned(Gimbal_GetCommandY());
        LogUart_SendString(" vision_ff_sps=");
        LogUart_SendSigned(Gimbal_GetVisionFeedForwardX());
        LogUart_SendString(",");
        LogUart_SendSigned(Gimbal_GetVisionFeedForwardY());
        LogUart_SendString(" step_sps=");
        LogUart_SendSigned(Motor_GetGimbalStepRate(MOTOR_GIMBAL_1));
        LogUart_SendString(",");
        LogUart_SendSigned(Motor_GetGimbalStepRate(MOTOR_GIMBAL_2));
        LogUart_SendString(" plot=");
        LogUart_SendUnsigned(g_visionPlotEnabled);
        LogUart_SendString("\r\n");
        return;
    }

    EncoderMotor_GetSnapshot(&snapshot);
    LogUart_SendString("D mode=");
    LogUart_SendString(TuningConsole_GetModeName(snapshot.mode));
    LogUart_SendString(" set=");
    LogUart_SendString(TuningConsole_GetSetStateName());
    LogUart_SendString(" gray=");
    LogUart_SendUnsigned(g_grayActive);
    LogUart_SendString(" gray_mask=");
    LogUart_SendUnsigned(g_grayMask);
    LogUart_SendString(" startup=");
    LogUart_SendUnsigned(snapshot.startupActive[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendUnsigned(snapshot.startupActive[ENCODER_MOTOR_RIGHT]);
    LogUart_SendString(" pwm_pct=");
    LogUart_SendSigned(TuningConsole_PwmToPercent(
        snapshot.outputPwm[ENCODER_MOTOR_LEFT]));
    LogUart_SendString(",");
    LogUart_SendSigned(TuningConsole_PwmToPercent(
        snapshot.outputPwm[ENCODER_MOTOR_RIGHT]));
    LogUart_SendString(" pwm_raw=");
    LogUart_SendSigned(snapshot.outputPwm[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot.outputPwm[ENCODER_MOTOR_RIGHT]);
    LogUart_SendString(" target=");
    LogUart_SendSigned(snapshot.targetCounts[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot.targetCounts[ENCODER_MOTOR_RIGHT]);
    LogUart_SendString(" count=");
    LogUart_SendSigned(snapshot.feedbackCounts[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot.feedbackCounts[ENCODER_MOTOR_RIGHT]);
    LogUart_SendString(" avg=");
    LogUart_SendSigned(TuningConsole_GetAverage(ENCODER_MOTOR_LEFT));
    LogUart_SendString(",");
    LogUart_SendSigned(TuningConsole_GetAverage(ENCODER_MOTOR_RIGHT));
    LogUart_SendString(" sum=");
    LogUart_SendSigned((int32_t)g_feedbackSum[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendSigned((int32_t)g_feedbackSum[ENCODER_MOTOR_RIGHT]);
    LogUart_SendString(" pwm_a_cc=");
    LogUart_SendUnsigned(
        snapshot.pwmCompareCounts[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(" irq_pa13=");
    LogUart_SendUnsigned(snapshot.rightEncoderAInterruptCount);
    LogUart_SendString(" irq_pb24=");
    LogUart_SendUnsigned(snapshot.rightEncoderBInterruptCount);
    LogUart_SendString(" level_pa13=");
    LogUart_SendUnsigned(snapshot.rightEncoderALevel);
    LogUart_SendString(" level_pb24=");
    LogUart_SendUnsigned(snapshot.rightEncoderBLevel);
    LogUart_SendString(" n=");
    LogUart_SendUnsigned(g_averageSampleCount);
    LogUart_SendString("\r\n");
    TuningConsole_SendPlotFrame(&snapshot);
}

static void TuningConsole_SendGimbalPlotFrame(void)
{
    GimbalAttitudeSnapshot gimbal;

    GimbalAttitude_GetSnapshot(&gimbal);
    LogUart_SendString("A ");
    LogUart_SendSigned(gimbal.feedbackYawX100);
    LogUart_SendString(",");
    LogUart_SendSigned(gimbal.referenceYawX100);
    LogUart_SendString(",");
    LogUart_SendSigned(gimbal.angleErrorX100);
    LogUart_SendString(",");
    LogUart_SendSigned(gimbal.feedbackRateX100PerSec);
    LogUart_SendString(",");
    LogUart_SendSigned(gimbal.motion.yawRateFilteredX100PerSec);
    LogUart_SendString(",");
    LogUart_SendSigned(gimbal.rateReferenceX100PerSec);
    LogUart_SendString(",");
    LogUart_SendSigned(gimbal.feedForwardSps);
    LogUart_SendString(",");
    LogUart_SendSigned(gimbal.commandSps);
    LogUart_SendString(",");
    LogUart_SendSigned(gimbal.stepOutputSps);
    LogUart_SendString("\r\n");
}

/* B: each axis shows raw error, adjusted error, command, STEP rate/count. */
static void TuningConsole_SendVisionPlotFrame(void)
{
    LogUart_SendString("B ");
    LogUart_SendSigned(Vision_GetRawX());
    LogUart_SendString(",");
    LogUart_SendSigned(Gimbal_GetErrorX());
    LogUart_SendString(",");
    LogUart_SendSigned(Gimbal_GetCommandX());
    LogUart_SendString(",");
    LogUart_SendSigned(Motor_GetGimbalStepRate(MOTOR_GIMBAL_1));
    LogUart_SendString(",");
    LogUart_SendSigned(Motor_GetStepCount(MOTOR_GIMBAL_1));
    LogUart_SendString(",");
    LogUart_SendSigned(Vision_GetRawY());
    LogUart_SendString(",");
    LogUart_SendSigned(Gimbal_GetErrorY());
    LogUart_SendString(",");
    LogUart_SendSigned(Gimbal_GetCommandY());
    LogUart_SendString(",");
    LogUart_SendSigned(Motor_GetGimbalStepRate(MOTOR_GIMBAL_2));
    LogUart_SendString(",");
    LogUart_SendSigned(Motor_GetStepCount(MOTOR_GIMBAL_2));
    LogUart_SendString("\r\n");
}

static void TuningConsole_SendDetails(void)
{
    EncoderMotorSnapshot snapshot;

    if (g_tuningMode == TUNING_MODE_GIMBAL) {
        GimbalAttitudeConfig gimbal;
        BodyMotionConfig motion;

        GimbalAttitude_GetConfig(&gimbal);
        BodyMotion_GetConfig(&motion);
        TuningConsole_SendStatus();
        LogUart_SendString("# gsteps=");
        LogUart_SendUnsigned(gimbal.stepsPerRevolution);
        LogUart_SendString(" gsign=");
        LogUart_SendSigned(gimbal.motorDirectionSign);
        LogUart_SendString(" gh7sign=");
        LogUart_SendSigned(gimbal.h7FeedbackSign);
        LogUart_SendString(" gjysign=");
        LogUart_SendSigned(gimbal.jy61FeedForwardSign);
        LogUart_SendString(" gkff=");
        LogUart_SendUnsigned(gimbal.jy61KffQ1024);
        LogUart_SendString(" gkp=");
        LogUart_SendUnsigned(gimbal.h7AngleKpQ1024);
        LogUart_SendString(" grkp=");
        LogUart_SendUnsigned(gimbal.h7RateKpQ1024);
        LogUart_SendString(" gmax=");
        LogUart_SendUnsigned(gimbal.maxSpeedSps);
        LogUart_SendString(" gaccel=");
        LogUart_SendUnsigned(gimbal.accelStepSps);
        LogUart_SendString(" glimit=");
        LogUart_SendUnsigned(gimbal.positionLimitSteps);
        LogUart_SendString("\r\n# glpf=");
        LogUart_SendUnsigned(motion.gyroAlphaQ1024);
        LogUart_SendString(" gbeta=");
        LogUart_SendUnsigned(motion.yawBetaQ1024);
        LogUart_SendString(" gpred_ms=");
        LogUart_SendUnsigned(motion.predictionMs);
        LogUart_SendString(" stale_ms=");
        LogUart_SendUnsigned(motion.staleMs);
        LogUart_SendString("\r\n");
        return;
    }
    if (g_tuningMode == TUNING_MODE_VISION) {
        const StaticConfigGimbalTask *config =
            StaticConfig_GetActiveGimbal();

        TuningConsole_SendStatus();
        LogUart_SendString("# deadband=");
        LogUart_SendUnsigned(config->deadbandX);
        LogUart_SendString(",");
        LogUart_SendUnsigned(config->deadbandY);
        LogUart_SendString(" restart=");
        LogUart_SendUnsigned(config->restartDeadbandX);
        LogUart_SendString(",");
        LogUart_SendUnsigned(config->restartDeadbandY);
        LogUart_SendString(" kp=");
        LogUart_SendUnsigned(config->kpX);
        LogUart_SendString(",");
        LogUart_SendUnsigned(config->kpY);
        LogUart_SendString(" kd=");
        LogUart_SendUnsigned(config->kdX);
        LogUart_SendString(",");
        LogUart_SendUnsigned(config->kdY);
        LogUart_SendString(" kff=");
        LogUart_SendUnsigned(config->kffX);
        LogUart_SendString(",");
        LogUart_SendUnsigned(config->kffY);
        LogUart_SendString(" speed_min=");
        LogUart_SendUnsigned(config->minSpeedX);
        LogUart_SendString(",");
        LogUart_SendUnsigned(config->minSpeedY);
        LogUart_SendString(" speed_max=");
        LogUart_SendUnsigned(config->maxSpeedX);
        LogUart_SendString(",");
        LogUart_SendUnsigned(config->maxSpeedY);
        LogUart_SendString(" offset=");
        LogUart_SendSigned(config->offsetX);
        LogUart_SendString(",");
        LogUart_SendSigned(config->offsetY);
        LogUart_SendString("\r\n");
        return;
    }

    EncoderMotor_GetSnapshot(&snapshot);
    LogUart_SendString("# mode=");
    LogUart_SendString(TuningConsole_GetModeName(snapshot.mode));
    LogUart_SendString(" period_ms=");
    LogUart_SendUnsigned(CHASSIS_CONTROL_PERIOD_MS);
    LogUart_SendString(" pwm_limit=");
    LogUart_SendUnsigned(CHASSIS_PWM_LIMIT_COUNTS);
    LogUart_SendString(" set=");
    LogUart_SendString(TuningConsole_GetSetStateName());
    LogUart_SendString(" gray=");
    LogUart_SendUnsigned(g_grayActive);
    LogUart_SendString(" gray_mask=");
    LogUart_SendUnsigned(g_grayMask);
    LogUart_SendString("\r\n");
    TuningConsole_SendPair("pwm_pct",
        TuningConsole_PwmToPercent(snapshot.outputPwm[0]),
        TuningConsole_PwmToPercent(snapshot.outputPwm[1]));
    TuningConsole_SendPair("pwm_raw", snapshot.outputPwm[0],
        snapshot.outputPwm[1]);
    TuningConsole_SendPair("pwm_cc",
        (int32_t)snapshot.pwmCompareCounts[0],
        (int32_t)snapshot.pwmCompareCounts[1]);
    TuningConsole_SendPair("target_count", snapshot.targetCounts[0],
        snapshot.targetCounts[1]);
    TuningConsole_SendPair("feedback_count", snapshot.feedbackCounts[0],
        snapshot.feedbackCounts[1]);
    TuningConsole_SendPair("average_count",
        TuningConsole_GetAverage(0U), TuningConsole_GetAverage(1U));
    TuningConsole_SendPair("feedback_sum",
        (int32_t)g_feedbackSum[0], (int32_t)g_feedbackSum[1]);
    TuningConsole_SendUnsignedPair("right_irq_pa13_pb24",
        snapshot.rightEncoderAInterruptCount,
        snapshot.rightEncoderBInterruptCount);
    TuningConsole_SendPair("right_level_pa13_pb24",
        snapshot.rightEncoderALevel, snapshot.rightEncoderBLevel);
    TuningConsole_SendPair("start_pct",
        TuningConsole_PwmToPercent(snapshot.tuning[0].startPwm),
        TuningConsole_PwmToPercent(snapshot.tuning[1].startPwm));
    TuningConsole_SendPair("start_raw", snapshot.tuning[0].startPwm,
        snapshot.tuning[1].startPwm);
    TuningConsole_SendPair("runstart_pct",
        TuningConsole_PwmToPercent(snapshot.tuning[0].runStartPwm),
        TuningConsole_PwmToPercent(snapshot.tuning[1].runStartPwm));
    TuningConsole_SendPair("runstart_raw",
        snapshot.tuning[0].runStartPwm,
        snapshot.tuning[1].runStartPwm);
    TuningConsole_SendPair("startup_active", snapshot.startupActive[0],
        snapshot.startupActive[1]);
    TuningConsole_SendPair("ff_q1024", snapshot.tuning[0].ffQ1024,
        snapshot.tuning[1].ffQ1024);
    TuningConsole_SendPair("kp_q1024", snapshot.tuning[0].kpQ1024,
        snapshot.tuning[1].kpQ1024);
    TuningConsole_SendPair("ki_q1024", snapshot.tuning[0].kiQ1024,
        snapshot.tuning[1].kiQ1024);
    TuningConsole_SendPair("ilim_pct",
        TuningConsole_PwmToPercent(snapshot.tuning[0].integralLimitPwm),
        TuningConsole_PwmToPercent(snapshot.tuning[1].integralLimitPwm));
    TuningConsole_SendPair("ilim_raw",
        snapshot.tuning[0].integralLimitPwm,
        snapshot.tuning[1].integralLimitPwm);
    TuningConsole_SendPair("integral_pct",
        TuningConsole_PwmToPercent(snapshot.integralOutputPwm[0]),
        TuningConsole_PwmToPercent(snapshot.integralOutputPwm[1]));
    TuningConsole_SendPair("integral_raw", snapshot.integralOutputPwm[0],
        snapshot.integralOutputPwm[1]);
    LogUart_SendString("# samples=");
    LogUart_SendUnsigned(g_averageSampleCount);
    LogUart_SendString(" rx_drop=");
    LogUart_SendUnsigned(LogUart_GetRxDropCount());
    LogUart_SendString(" rx_error=");
    LogUart_SendUnsigned(LogUart_GetRxErrorCount());
    LogUart_SendString("\r\n");
}

static void TuningConsole_ApplyGain(const char *command, int32_t left,
    int32_t right)
{
    EncoderMotorSnapshot snapshot;

    if ((left < 0) || (right < 0)) {
        LogUart_SendString("#ERR gain must be non-negative\r\n");
        return;
    }
    EncoderMotor_GetSnapshot(&snapshot);
    if (TuningConsole_TextEquals(command, "ff") != 0U) {
        snapshot.tuning[0].ffQ1024 = left;
        snapshot.tuning[1].ffQ1024 = right;
    } else if (TuningConsole_TextEquals(command, "kp") != 0U) {
        snapshot.tuning[0].kpQ1024 = left;
        snapshot.tuning[1].kpQ1024 = right;
    } else {
        snapshot.tuning[0].kiQ1024 = left;
        snapshot.tuning[1].kiQ1024 = right;
    }
    EncoderMotor_SetTuning(&snapshot.tuning[0], &snapshot.tuning[1]);
    TuningConsole_SendPair(command, left, right);
    g_forceStatus = 1U;
}

static void TuningConsole_ApplyPwmParameter(const char *command,
    int32_t left, int32_t right)
{
    EncoderMotorSnapshot snapshot;
    int16_t leftRaw;
    int16_t rightRaw;

    if ((left < 0) || (left > TUNING_PWM_PERCENT_MAX) ||
        (right < 0) || (right > TUNING_PWM_PERCENT_MAX)) {
        LogUart_SendString("#ERR range is 0..100 percent\r\n");
        return;
    }
    leftRaw = TuningConsole_PercentToPwm(left);
    rightRaw = TuningConsole_PercentToPwm(right);
    EncoderMotor_GetSnapshot(&snapshot);
    if (TuningConsole_TextEquals(command, "start") != 0U) {
        snapshot.tuning[0].startPwm = leftRaw;
        snapshot.tuning[1].startPwm = rightRaw;
        TuningConsole_SendPair("start_pct", left, right);
        TuningConsole_SendPair("start_raw", leftRaw, rightRaw);
    } else if (TuningConsole_TextEquals(command, "runstart") != 0U) {
        snapshot.tuning[0].runStartPwm = leftRaw;
        snapshot.tuning[1].runStartPwm = rightRaw;
        TuningConsole_SendPair("runstart_pct", left, right);
        TuningConsole_SendPair("runstart_raw", leftRaw, rightRaw);
    } else {
        snapshot.tuning[0].integralLimitPwm = leftRaw;
        snapshot.tuning[1].integralLimitPwm = rightRaw;
        TuningConsole_SendPair("ilim_pct", left, right);
        TuningConsole_SendPair("ilim_raw", leftRaw, rightRaw);
    }
    EncoderMotor_SetTuning(&snapshot.tuning[0], &snapshot.tuning[1]);
    g_forceStatus = 1U;
}

static void TuningConsole_CalculateFf(char *tokens[])
{
    int32_t percent1;
    int32_t pwm1;
    int32_t count1;
    int32_t percent2;
    int32_t pwm2;
    int32_t count2;
    int32_t countDelta;
    int64_t result;
    int64_t interceptScaled;
    int32_t runStartPwm;

    if ((TuningConsole_ParseInt32(tokens[1], &percent1) == 0U) ||
        (TuningConsole_ParseInt32(tokens[2], &count1) == 0U) ||
        (TuningConsole_ParseInt32(tokens[3], &percent2) == 0U) ||
        (TuningConsole_ParseInt32(tokens[4], &count2) == 0U)) {
        LogUart_SendString("#ERR ffcalc expects four integers\r\n");
        return;
    }
    if ((percent1 < -TUNING_PWM_PERCENT_MAX) ||
        (percent1 > TUNING_PWM_PERCENT_MAX) ||
        (percent2 < -TUNING_PWM_PERCENT_MAX) ||
        (percent2 > TUNING_PWM_PERCENT_MAX)) {
        LogUart_SendString("#ERR ffcalc PWM must be -100..100 percent\r\n");
        return;
    }
    pwm1 = TuningConsole_PercentToPwm(percent1);
    pwm2 = TuningConsole_PercentToPwm(percent2);
    countDelta = count2 - count1;
    if (countDelta == 0) {
        LogUart_SendString("#ERR count2 must differ from count1\r\n");
        return;
    }
    result = ((int64_t)pwm2 - pwm1) * CHASSIS_Q1024_SCALE /
        countDelta;
    if ((result < 0) || (result > 0x7FFFFFFFLL)) {
        LogUart_SendString("#ERR FF result must be positive int32\r\n");
        return;
    }
    interceptScaled = (int64_t)pwm1 * CHASSIS_Q1024_SCALE -
        result * count1;
    if (interceptScaled < 0) {
        interceptScaled = -interceptScaled;
    }
    runStartPwm = (int32_t)((interceptScaled +
        CHASSIS_Q1024_SCALE / 2) / CHASSIS_Q1024_SCALE);
    if (runStartPwm > (int32_t)CHASSIS_PWM_LIMIT_COUNTS) {
        LogUart_SendString("#ERR runstart result exceeds PWM limit\r\n");
        return;
    }
    TuningConsole_SendPair("ffcalc_pwm_raw", pwm1, pwm2);
    LogUart_SendString("# ff_q1024=");
    LogUart_SendSigned((int32_t)result);
    LogUart_SendString(" runstart_raw=");
    LogUart_SendSigned(runStartPwm);
    LogUart_SendString(" runstart_pct=");
    LogUart_SendSigned(TuningConsole_PwmToPercent((int16_t)runStartPwm));
    LogUart_SendString("\r\n");
    LogUart_SendString("FITCALC,");
    LogUart_SendSigned(percent1);
    LogUart_SendString(",");
    LogUart_SendSigned(pwm1);
    LogUart_SendString(",");
    LogUart_SendSigned(count1);
    LogUart_SendString(",");
    LogUart_SendSigned(percent2);
    LogUart_SendString(",");
    LogUart_SendSigned(pwm2);
    LogUart_SendString(",");
    LogUart_SendSigned(count2);
    LogUart_SendString(",");
    LogUart_SendSigned((int32_t)result);
    LogUart_SendString(",");
    LogUart_SendSigned(runStartPwm);
    LogUart_SendString("\r\n");
    TuningConsole_SendPlotFitFrame(0U, pwm1, count1,
        pwm2, count2, (int32_t)result, 0, 0);
}

static void TuningConsole_ClearSetReferences(void)
{
    uint8_t motorIndex;

    for (motorIndex = 0U; motorIndex < ENCODER_MOTOR_COUNT; ++motorIndex) {
        g_setReference[motorIndex].pwm = 0;
        g_setReference[motorIndex].feedbackSum = 0;
        g_setReference[motorIndex].sampleCount = 0U;
        g_setReference[motorIndex].valid = 0U;
        g_setResult[motorIndex] = TUNING_CONSOLE_SET_RESULT_NONE;
    }
}

static void TuningConsole_CancelSetSampling(void)
{
    uint8_t motorIndex;

    g_setState = TUNING_SET_IDLE;
    g_setPwm[ENCODER_MOTOR_LEFT] = 0;
    g_setPwm[ENCODER_MOTOR_RIGHT] = 0;
    for (motorIndex = 0U; motorIndex < ENCODER_MOTOR_COUNT; ++motorIndex) {
        if (g_setResult[motorIndex] ==
            TUNING_CONSOLE_SET_RESULT_RUNNING) {
            g_setResult[motorIndex] = TUNING_CONSOLE_SET_RESULT_NONE;
        }
    }
}

static void TuningConsole_CancelEncoderMove(void)
{
    g_encoderMoveActive = 0U;
    g_encoderMoveDistance[ENCODER_MOTOR_LEFT] = 0;
    g_encoderMoveDistance[ENCODER_MOTOR_RIGHT] = 0;
}

static int16_t TuningConsole_GetEncoderMoveTarget(uint8_t motorIndex)
{
    if (g_encoderMoveDistance[motorIndex] > 0) {
        return g_encoderMoveSpeed[motorIndex];
    }
    if (g_encoderMoveDistance[motorIndex] < 0) {
        return (int16_t)(-g_encoderMoveSpeed[motorIndex]);
    }
    return 0;
}

static int64_t TuningConsole_GetEncoderMoveDelta(uint8_t motorIndex)
{
    return (int64_t)EncoderMotor_GetTotalCount(motorIndex) -
        (int64_t)g_encoderMoveStart[motorIndex];
}

static uint8_t TuningConsole_EncoderMoveReached(uint8_t motorIndex)
{
    int64_t delta = TuningConsole_GetEncoderMoveDelta(motorIndex);
    int32_t distance = g_encoderMoveDistance[motorIndex];

    if (distance > 0) {
        return (delta >= (int64_t)distance) ? 1U : 0U;
    }
    if (distance < 0) {
        return (delta <= (int64_t)distance) ? 1U : 0U;
    }
    return 1U;
}

static void TuningConsole_StartEncoderMove(int32_t leftSpeed,
    int32_t rightSpeed, int32_t leftDistance, int32_t rightDistance)
{
    int16_t leftTarget;
    int16_t rightTarget;

    if ((leftSpeed < 1) ||
        (leftSpeed > TUNING_ENCODER_SPEED_MAX_COUNTS) ||
        (rightSpeed < 1) ||
        (rightSpeed > TUNING_ENCODER_SPEED_MAX_COUNTS)) {
        LogUart_SendString(
            "#ERR move speed range is 1..100 count/20ms\r\n");
        return;
    }
    if ((leftDistance < -TUNING_ENCODER_DISTANCE_LIMIT) ||
        (leftDistance > TUNING_ENCODER_DISTANCE_LIMIT) ||
        (rightDistance < -TUNING_ENCODER_DISTANCE_LIMIT) ||
        (rightDistance > TUNING_ENCODER_DISTANCE_LIMIT) ||
        ((leftDistance == 0) && (rightDistance == 0))) {
        LogUart_SendString(
            "#ERR move distance range is -1000000000..1000000000, one wheel nonzero\r\n");
        return;
    }

    TuningConsole_CancelSetSampling();
    TuningConsole_CancelEncoderMove();
    TuningConsole_StopGrayTest();
    g_encoderMoveSpeed[ENCODER_MOTOR_LEFT] = (int16_t)leftSpeed;
    g_encoderMoveSpeed[ENCODER_MOTOR_RIGHT] = (int16_t)rightSpeed;
    g_encoderMoveStart[ENCODER_MOTOR_LEFT] =
        EncoderMotor_GetTotalCount(ENCODER_MOTOR_LEFT);
    g_encoderMoveStart[ENCODER_MOTOR_RIGHT] =
        EncoderMotor_GetTotalCount(ENCODER_MOTOR_RIGHT);
    g_encoderMoveDistance[ENCODER_MOTOR_LEFT] = leftDistance;
    g_encoderMoveDistance[ENCODER_MOTOR_RIGHT] = rightDistance;
    g_encoderMoveActive = 1U;
    leftTarget = TuningConsole_GetEncoderMoveTarget(ENCODER_MOTOR_LEFT);
    rightTarget = TuningConsole_GetEncoderMoveTarget(ENCODER_MOTOR_RIGHT);
    EncoderMotor_SetCalibrationTargets(leftTarget, rightTarget);
    g_oledPage = TUNING_CONSOLE_OLED_PID;
    TuningConsole_ResetAverage();
    TuningConsole_SendPair("move_speed", leftSpeed, rightSpeed);
    TuningConsole_SendPair("move_distance", leftDistance, rightDistance);
    g_forceStatus = 1U;
}

static void TuningConsole_UpdateEncoderMove(void)
{
    int16_t leftTarget;
    int16_t rightTarget;
    int32_t leftDelta;
    int32_t rightDelta;
    uint8_t targetChanged = 0U;

    if (g_encoderMoveActive == 0U) {
        return;
    }

    if ((g_encoderMoveDistance[ENCODER_MOTOR_LEFT] != 0) &&
        (TuningConsole_EncoderMoveReached(ENCODER_MOTOR_LEFT) != 0U)) {
        g_encoderMoveDistance[ENCODER_MOTOR_LEFT] = 0;
        targetChanged = 1U;
    }
    if ((g_encoderMoveDistance[ENCODER_MOTOR_RIGHT] != 0) &&
        (TuningConsole_EncoderMoveReached(ENCODER_MOTOR_RIGHT) != 0U)) {
        g_encoderMoveDistance[ENCODER_MOTOR_RIGHT] = 0;
        targetChanged = 1U;
    }
    if (targetChanged == 0U) {
        return;
    }

    leftTarget = TuningConsole_GetEncoderMoveTarget(ENCODER_MOTOR_LEFT);
    rightTarget = TuningConsole_GetEncoderMoveTarget(ENCODER_MOTOR_RIGHT);
    EncoderMotor_SetCalibrationTargets(leftTarget, rightTarget);
    if ((leftTarget != 0) || (rightTarget != 0)) {
        g_forceStatus = 1U;
        return;
    }

    leftDelta = (int32_t)TuningConsole_GetEncoderMoveDelta(
        ENCODER_MOTOR_LEFT);
    rightDelta = (int32_t)TuningConsole_GetEncoderMoveDelta(
        ENCODER_MOTOR_RIGHT);
    g_encoderMoveActive = 0U;
    TuningConsole_SendPair("move_done_delta", leftDelta, rightDelta);
    g_forceStatus = 1U;
}

static void TuningConsole_SendSetPoint(uint8_t motorIndex,
    const TuningSetPoint *point)
{
    const char *name = (motorIndex == ENCODER_MOTOR_LEFT) ? "left" :
        "right";
    int32_t averageX1000 = (int32_t)((point->feedbackSum * 1000) /
        (int64_t)point->sampleCount);

    LogUart_SendString("#SET ");
    LogUart_SendString(name);
    LogUart_SendString(" pwm_pct=");
    LogUart_SendSigned(TuningConsole_PwmToPercent(point->pwm));
    LogUart_SendString(" pwm_raw=");
    LogUart_SendSigned(point->pwm);
    LogUart_SendString(" sum=");
    LogUart_SendSigned((int32_t)point->feedbackSum);
    LogUart_SendString(" n=");
    LogUart_SendUnsigned(point->sampleCount);
    LogUart_SendString(" avg_x1000=");
    LogUart_SendSigned(averageX1000);
    LogUart_SendString("\r\n");
}

static void TuningConsole_SendFitRecord(uint8_t motorIndex,
    const TuningSetPoint *point1, const TuningSetPoint *point2, int32_t ff,
    int16_t runStartPwm, int32_t leftFfQ1024, int32_t rightFfQ1024)
{
    const char *name = (motorIndex == ENCODER_MOTOR_LEFT) ? "left" :
        "right";
    int32_t average1X1000 = (int32_t)((point1->feedbackSum * 1000) /
        (int64_t)point1->sampleCount);
    int32_t average2X1000 = (int32_t)((point2->feedbackSum * 1000) /
        (int64_t)point2->sampleCount);

    LogUart_SendString("FIT,");
    LogUart_SendString(name);
    LogUart_SendString(",");
    LogUart_SendSigned(TuningConsole_PwmToPercent(point1->pwm));
    LogUart_SendString(",");
    LogUart_SendSigned(point1->pwm);
    LogUart_SendString(",");
    LogUart_SendSigned((int32_t)point1->feedbackSum);
    LogUart_SendString(",");
    LogUart_SendUnsigned(point1->sampleCount);
    LogUart_SendString(",");
    LogUart_SendSigned(average1X1000);
    LogUart_SendString(",");
    LogUart_SendSigned(TuningConsole_PwmToPercent(point2->pwm));
    LogUart_SendString(",");
    LogUart_SendSigned(point2->pwm);
    LogUart_SendString(",");
    LogUart_SendSigned((int32_t)point2->feedbackSum);
    LogUart_SendString(",");
    LogUart_SendUnsigned(point2->sampleCount);
    LogUart_SendString(",");
    LogUart_SendSigned(average2X1000);
    LogUart_SendString(",");
    LogUart_SendSigned(ff);
    LogUart_SendString(",");
    LogUart_SendSigned(runStartPwm);
    LogUart_SendString("\r\n");
    TuningConsole_SendPlotFitFrame((uint8_t)(motorIndex + 1U),
        point1->pwm, average1X1000, point2->pwm, average2X1000, ff,
        leftFfQ1024, rightFfQ1024);
}

static uint8_t TuningConsole_ApplySetPoint(uint8_t motorIndex,
    const TuningSetPoint *point, EncoderMotorSnapshot *snapshot)
{
    TuningSetPoint *reference = &g_setReference[motorIndex];
    const char *name = (motorIndex == ENCODER_MOTOR_LEFT) ? "left" :
        "right";
    int64_t numerator;
    int64_t denominator;
    int64_t ff;
    int64_t interceptNumerator;
    int64_t interceptDenominator;
    int64_t runStartPwm;

    TuningConsole_SendSetPoint(motorIndex, point);
    if (((point->pwm > 0) && (point->feedbackSum <= 0)) ||
        ((point->pwm < 0) && (point->feedbackSum >= 0))) {
        g_setResult[motorIndex] = TUNING_CONSOLE_SET_RESULT_ERROR;
        LogUart_SendString("#ERR set ");
        LogUart_SendString(name);
        LogUart_SendString(" count sign mismatch; check PWM/direction\r\n");
        return 0U;
    }
    if (reference->valid == 0U) {
        *reference = *point;
        reference->valid = 1U;
        g_setResult[motorIndex] =
            TUNING_CONSOLE_SET_RESULT_POINT_STORED;
        LogUart_SendString("#SET ");
        LogUart_SendString(name);
        LogUart_SendString(" point1 stored; send next set point\r\n");
        return 0U;
    }
    if (reference->pwm == point->pwm) {
        g_setResult[motorIndex] = TUNING_CONSOLE_SET_RESULT_ERROR;
        LogUart_SendString("#ERR set ");
        LogUart_SendString(name);
        LogUart_SendString(" second PWM must differ\r\n");
        return 0U;
    }
    if (((int32_t)reference->pwm * point->pwm) < 0) {
        g_setResult[motorIndex] = TUNING_CONSOLE_SET_RESULT_ERROR;
        LogUart_SendString("#ERR set ");
        LogUart_SendString(name);
        LogUart_SendString(" two FF points must use same direction\r\n");
        return 0U;
    }

    numerator = ((int64_t)point->pwm - reference->pwm) *
        CHASSIS_Q1024_SCALE * reference->sampleCount *
        point->sampleCount;
    denominator = point->feedbackSum * reference->sampleCount -
        reference->feedbackSum * point->sampleCount;
    if (denominator == 0) {
        g_setResult[motorIndex] = TUNING_CONSOLE_SET_RESULT_ERROR;
        LogUart_SendString("#ERR set ");
        LogUart_SendString(name);
        LogUart_SendString(" average counts did not change\r\n");
        return 0U;
    }
    if (denominator < 0) {
        denominator = -denominator;
        numerator = -numerator;
    }
    if (numerator <= 0) {
        g_setResult[motorIndex] = TUNING_CONSOLE_SET_RESULT_ERROR;
        LogUart_SendString("#ERR set ");
        LogUart_SendString(name);
        LogUart_SendString(" PWM/count slope must be positive\r\n");
        return 0U;
    }
    ff = (numerator + denominator / 2) / denominator;
    if (ff > 0x7FFFFFFFLL) {
        g_setResult[motorIndex] = TUNING_CONSOLE_SET_RESULT_ERROR;
        LogUart_SendString("#ERR set FF exceeds int32\r\n");
        return 0U;
    }
    interceptNumerator =
        (int64_t)reference->pwm * CHASSIS_Q1024_SCALE *
        reference->sampleCount - ff * reference->feedbackSum;
    if (interceptNumerator < 0) {
        interceptNumerator = -interceptNumerator;
    }
    interceptDenominator =
        (int64_t)CHASSIS_Q1024_SCALE * reference->sampleCount;
    runStartPwm = (interceptNumerator + interceptDenominator / 2) /
        interceptDenominator;
    if (runStartPwm > CHASSIS_PWM_LIMIT_COUNTS) {
        g_setResult[motorIndex] = TUNING_CONSOLE_SET_RESULT_ERROR;
        LogUart_SendString("#ERR set runstart exceeds PWM limit\r\n");
        return 0U;
    }

    snapshot->tuning[motorIndex].ffQ1024 = (int32_t)ff;
    snapshot->tuning[motorIndex].runStartPwm = (int16_t)runStartPwm;
    g_setResult[motorIndex] = TUNING_CONSOLE_SET_RESULT_FF_APPLIED;
    TuningConsole_SendFitRecord(motorIndex, reference, point,
        (int32_t)ff, (int16_t)runStartPwm,
        snapshot->tuning[ENCODER_MOTOR_LEFT].ffQ1024,
        snapshot->tuning[ENCODER_MOTOR_RIGHT].ffQ1024);
    *reference = *point;
    reference->valid = 1U;
    LogUart_SendString("#SET ");
    LogUart_SendString(name);
    LogUart_SendString(" ff_q1024=");
    LogUart_SendSigned((int32_t)ff);
    LogUart_SendString(" runstart_raw=");
    LogUart_SendSigned((int32_t)runStartPwm);
    LogUart_SendString(" applied\r\n");
    return 1U;
}

static void TuningConsole_FinishSetSampling(void)
{
    EncoderMotorSnapshot snapshot;
    TuningSetPoint point;
    uint8_t motorIndex;
    uint8_t applyTuning = 0U;

    EncoderMotor_SetOpenLoopPwm(0, 0);
    EncoderMotor_GetSnapshot(&snapshot);
    for (motorIndex = 0U; motorIndex < ENCODER_MOTOR_COUNT; ++motorIndex) {
        if (g_setPwm[motorIndex] == 0) {
            continue;
        }
        point.pwm = g_setPwm[motorIndex];
        point.feedbackSum = g_feedbackSum[motorIndex];
        point.sampleCount = g_averageSampleCount;
        point.valid = 1U;
        if (TuningConsole_ApplySetPoint(motorIndex, &point,
            &snapshot) != 0U) {
            applyTuning = 1U;
        }
    }
    if (applyTuning != 0U) {
        EncoderMotor_SetTuning(&snapshot.tuning[ENCODER_MOTOR_LEFT],
            &snapshot.tuning[ENCODER_MOTOR_RIGHT]);
    }
    TuningConsole_CancelSetSampling();
    TuningConsole_ResetAverage();
    LogUart_SendString("#SET done; PWM=0\r\n");
    g_forceStatus = 1U;
}

static void TuningConsole_UpdateSetSampling(TickType_t now)
{
    if (g_setState == TUNING_SET_SETTLING) {
        if ((now - g_setStateStartTime) <
            pdMS_TO_TICKS(TUNING_SET_SETTLE_MS)) {
            return;
        }
        TuningConsole_ResetAverage();
        g_setState = TUNING_SET_SAMPLING;
        LogUart_SendString("#SET sampling ");
        LogUart_SendUnsigned(TUNING_SET_SAMPLE_WINDOWS);
        LogUart_SendString(" x ");
        LogUart_SendUnsigned(CHASSIS_CONTROL_PERIOD_MS);
        LogUart_SendString("ms windows\r\n");
        g_forceStatus = 1U;
        return;
    }
    if ((g_setState == TUNING_SET_SAMPLING) &&
        (g_averageSampleCount >= TUNING_SET_SAMPLE_WINDOWS)) {
        TuningConsole_FinishSetSampling();
    }
}

static void TuningConsole_StartSetSampling(int32_t left, int32_t right)
{
    int16_t leftRaw;
    int16_t rightRaw;

    if (g_setState != TUNING_SET_IDLE) {
        LogUart_SendString("#ERR set busy; wait or send stop\r\n");
        return;
    }
    if ((left < -TUNING_PWM_PERCENT_MAX) ||
        (left > TUNING_PWM_PERCENT_MAX) ||
        (right < -TUNING_PWM_PERCENT_MAX) ||
        (right > TUNING_PWM_PERCENT_MAX) ||
        ((left == 0) && (right == 0))) {
        LogUart_SendString(
            "#ERR set range is -100..100%, one wheel nonzero\r\n");
        return;
    }

    TuningConsole_CancelEncoderMove();
    TuningConsole_StopGrayTest();
    leftRaw = TuningConsole_PercentToPwm(left);
    rightRaw = TuningConsole_PercentToPwm(right);
    g_setPwm[ENCODER_MOTOR_LEFT] = leftRaw;
    g_setPwm[ENCODER_MOTOR_RIGHT] = rightRaw;
    if (leftRaw != 0) {
        g_setResult[ENCODER_MOTOR_LEFT] =
            TUNING_CONSOLE_SET_RESULT_RUNNING;
    }
    if (rightRaw != 0) {
        g_setResult[ENCODER_MOTOR_RIGHT] =
            TUNING_CONSOLE_SET_RESULT_RUNNING;
    }
    EncoderMotor_SetOpenLoopPwm(leftRaw, rightRaw);
    TuningConsole_ResetAverage();
    g_setState = TUNING_SET_SETTLING;
    g_setStateStartTime = xTaskGetTickCount();
    LogUart_SendString("#SET pwm_pct=");
    LogUart_SendSigned(left);
    LogUart_SendString(",");
    LogUart_SendSigned(right);
    LogUart_SendString(" pwm_raw=");
    LogUart_SendSigned(leftRaw);
    LogUart_SendString(",");
    LogUart_SendSigned(rightRaw);
    LogUart_SendString(" settling ");
    LogUart_SendUnsigned(TUNING_SET_SETTLE_MS);
    LogUart_SendString("ms\r\n");
    g_forceStatus = 1U;
}

static uint8_t TuningConsole_ApplyGimbalValue(const char *command,
    int32_t value)
{
    GimbalAttitudeConfig gimbal;
    BodyMotionConfig motion;
    uint8_t motionParameter = 0U;

    GimbalAttitude_GetConfig(&gimbal);
    BodyMotion_GetConfig(&motion);
    if (TuningConsole_TextEquals(command, "gsteps") != 0U) {
        if ((value < 0) || (value > 65535L)) {
            return 0U;
        }
        gimbal.stepsPerRevolution = (uint16_t)value;
    } else if (TuningConsole_TextEquals(command, "gsign") != 0U) {
        if ((value != -1) && (value != 1)) {
            return 0U;
        }
        gimbal.motorDirectionSign = (int8_t)value;
    } else if (TuningConsole_TextEquals(command, "gh7sign") != 0U) {
        if ((value != -1) && (value != 1)) {
            return 0U;
        }
        gimbal.h7FeedbackSign = (int8_t)value;
    } else if (TuningConsole_TextEquals(command, "gjysign") != 0U) {
        if ((value != -1) && (value != 1)) {
            return 0U;
        }
        gimbal.jy61FeedForwardSign = (int8_t)value;
    } else if (TuningConsole_TextEquals(command, "gkff") != 0U) {
        if ((value < 0) || (value > 65535L)) {
            return 0U;
        }
        gimbal.jy61KffQ1024 = (uint16_t)value;
    } else if (TuningConsole_TextEquals(command, "gkp") != 0U) {
        if ((value < 0) || (value > 65535L)) {
            return 0U;
        }
        gimbal.h7AngleKpQ1024 = (uint16_t)value;
    } else if (TuningConsole_TextEquals(command, "grkp") != 0U) {
        if ((value < 0) || (value > 65535L)) {
            return 0U;
        }
        gimbal.h7RateKpQ1024 = (uint16_t)value;
    } else if (TuningConsole_TextEquals(command, "gmax") != 0U) {
        if ((value < 0) || (value > 65535L)) {
            return 0U;
        }
        gimbal.maxSpeedSps = (uint16_t)value;
    } else if (TuningConsole_TextEquals(command, "gaccel") != 0U) {
        if ((value < 0) || (value > 65535L)) {
            return 0U;
        }
        gimbal.accelStepSps = (uint16_t)value;
    } else if (TuningConsole_TextEquals(command, "glimit") != 0U) {
        if (value < 0) {
            return 0U;
        }
        gimbal.positionLimitSteps = (uint32_t)value;
    } else if (TuningConsole_TextEquals(command, "glpf") != 0U) {
        if ((value < 0) || (value > 65535L)) {
            return 0U;
        }
        motion.gyroAlphaQ1024 = (uint16_t)value;
        motionParameter = 1U;
    } else if (TuningConsole_TextEquals(command, "gbeta") != 0U) {
        if ((value < 0) || (value > 65535L)) {
            return 0U;
        }
        motion.yawBetaQ1024 = (uint16_t)value;
        motionParameter = 1U;
    } else if (TuningConsole_TextEquals(command, "gpred") != 0U) {
        if ((value < 0) || (value > 65535L)) {
            return 0U;
        }
        motion.predictionMs = (uint16_t)value;
        motionParameter = 1U;
    } else {
        return 0U;
    }

    if (((motionParameter != 0U) &&
        (BodyMotion_SetConfig(&motion) == 0U)) ||
        ((motionParameter == 0U) &&
        (GimbalAttitude_SetConfig(&gimbal) == 0U))) {
        return 0U;
    }
    LogUart_SendString("#OK ");
    LogUart_SendString(command);
    LogUart_SendString("=");
    LogUart_SendSigned(value);
    LogUart_SendString("\r\n");
    g_forceStatus = 1U;
    return 1U;
}

static uint8_t TuningConsole_ExecuteGimbalCommand(char *tokens[],
    uint8_t tokenCount)
{
    int32_t value;
    uint8_t recognized = (uint8_t)(
        (TuningConsole_TextEquals(tokens[0], "gshow") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gcal") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "ghold") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gff") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gplot") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gsteps") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gsign") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gh7sign") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gjysign") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gkff") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gkp") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "grkp") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gmax") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gaccel") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "glimit") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "glpf") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gbeta") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "gpred") != 0U));

    if (recognized == 0U) {
        return 0U;
    }
    if (g_tuningMode != TUNING_MODE_GIMBAL) {
        LogUart_SendString(
            "#ERR gimbal command blocked; send mode gimbal first\r\n");
        return 1U;
    }

    if ((TuningConsole_TextEquals(tokens[0], "gshow") != 0U) &&
        (tokenCount == 1U)) {
        TuningConsole_SendDetails();
        return 1U;
    }
    if ((TuningConsole_TextEquals(tokens[0], "gcal") != 0U) &&
        (tokenCount == 1U)) {
        if (GimbalAttitude_IsActive() == 0U) {
            GimbalAttitude_Start();
        } else {
            GimbalAttitude_StartCalibration();
        }
        g_forceStatus = 1U;
        LogUart_SendString(
            "#OK gcal started; keep board JY61 still\r\n");
        return 1U;
    }
    if ((TuningConsole_TextEquals(tokens[0], "ghold") != 0U) &&
        (tokenCount == 2U)) {
        if (TuningConsole_TextEquals(tokens[1], "on") != 0U) {
            if (GimbalAttitude_IsActive() == 0U) {
                GimbalAttitude_Start();
            } else {
                GimbalAttitude_SetHoldEnabled(1U);
            }
        } else if (TuningConsole_TextEquals(tokens[1], "off") != 0U) {
            GimbalAttitude_SetHoldEnabled(0U);
        } else {
            LogUart_SendString("#ERR ghold expects on or off\r\n");
            return 1U;
        }
        g_forceStatus = 1U;
        LogUart_SendString("#OK ghold=");
        LogUart_SendString(tokens[1]);
        LogUart_SendString("\r\n");
        return 1U;
    }
    if ((TuningConsole_TextEquals(tokens[0], "gff") != 0U) &&
        (tokenCount == 2U)) {
        if (TuningConsole_TextEquals(tokens[1], "on") != 0U) {
            GimbalAttitude_SetFeedForwardEnabled(1U);
        } else if (TuningConsole_TextEquals(tokens[1], "off") != 0U) {
            GimbalAttitude_SetFeedForwardEnabled(0U);
        } else {
            LogUart_SendString("#ERR gff expects on or off\r\n");
            return 1U;
        }
        g_forceStatus = 1U;
        LogUart_SendString("#OK gff=");
        LogUart_SendString(tokens[1]);
        LogUart_SendString("\r\n");
        return 1U;
    }
    if ((TuningConsole_TextEquals(tokens[0], "gplot") != 0U) &&
        (tokenCount == 2U)) {
        if (TuningConsole_TextEquals(tokens[1], "on") != 0U) {
            g_gimbalPlotEnabled = 1U;
            g_lastGimbalPlotTime = xTaskGetTickCount();
        } else if (TuningConsole_TextEquals(tokens[1], "off") != 0U) {
            g_gimbalPlotEnabled = 0U;
        } else {
            LogUart_SendString("#ERR gplot expects on or off\r\n");
            return 1U;
        }
        g_forceStatus = 1U;
        LogUart_SendString("#OK gplot=");
        LogUart_SendString(tokens[1]);
        LogUart_SendString("\r\n");
        return 1U;
    }
    if ((tokenCount == 2U) &&
        (TuningConsole_ParseInt32(tokens[1], &value) != 0U)) {
        if (TuningConsole_ApplyGimbalValue(tokens[0], value) == 0U) {
            LogUart_SendString("#ERR invalid gimbal parameter or range\r\n");
        }
        return 1U;
    }
    LogUart_SendString("#ERR bad gimbal command arguments\r\n");
    return 1U;
}

/* 把Task5视觉调试参数限制为最终保留的点/圆两种误差源。 */
static uint8_t TuningConsole_ParseVisionSource(const char *text,
    StaticConfigMode *source)
{
    if (TuningConsole_TextEquals(text, "center") != 0U) {
        *source = STATICCONFIG_MODE_CENTER;
    } else if (TuningConsole_TextEquals(text, "circle") != 0U) {
        *source = STATICCONFIG_MODE_CIRCLE;
    } else {
        return 0U;
    }
    return 1U;
}

/*
 * 处理Task5视觉命令；vconfig只允许center/circle并会重启视觉控制。
 * 不用于比赛任务切换，返回1仅表示命令名已被本处理器消费。
 */
static uint8_t TuningConsole_ExecuteVisionCommand(char *tokens[],
    uint8_t tokenCount)
{
    uint8_t recognized = (uint8_t)(
        (TuningConsole_TextEquals(tokens[0], "vshow") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "vplot") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "vconfig") != 0U));

    if (recognized == 0U) {
        return 0U;
    }
    if (g_tuningMode != TUNING_MODE_VISION) {
        LogUart_SendString(
            "#ERR vision command blocked; send mode vision first\r\n");
        return 1U;
    }
    if ((TuningConsole_TextEquals(tokens[0], "vshow") != 0U) &&
        (tokenCount == 1U)) {
        TuningConsole_SendDetails();
        return 1U;
    }
    if ((TuningConsole_TextEquals(tokens[0], "vplot") != 0U) &&
        (tokenCount == 2U)) {
        if (TuningConsole_TextEquals(tokens[1], "on") != 0U) {
            g_visionPlotEnabled = 1U;
            g_lastVisionPlotTime = xTaskGetTickCount();
        } else if (TuningConsole_TextEquals(tokens[1], "off") != 0U) {
            g_visionPlotEnabled = 0U;
        } else {
            LogUart_SendString("#ERR vplot expects on or off\r\n");
            return 1U;
        }
        LogUart_SendString("#OK vplot=");
        LogUart_SendString(tokens[1]);
        LogUart_SendString("\r\n");
        g_forceStatus = 1U;
        return 1U;
    }
    if ((TuningConsole_TextEquals(tokens[0], "vconfig") != 0U) &&
        (tokenCount == 2U)) {
        StaticConfigMode source;
        uint8_t plotEnabled = g_visionPlotEnabled;

        if (TuningConsole_ParseVisionSource(tokens[1], &source) == 0U) {
            LogUart_SendString(
                "#ERR vconfig expects center|circle\r\n");
            return 1U;
        }
        g_visionSource = source;
        TuningConsole_StopVisionActivity();
        TuningConsole_StartVisionActivity();
        g_visionPlotEnabled = plotEnabled;
        g_lastVisionPlotTime = xTaskGetTickCount();
        LogUart_SendString("#OK vconfig=");
        LogUart_SendString(tokens[1]);
        LogUart_SendString("\r\n");
        g_forceStatus = 1U;
        return 1U;
    }
    LogUart_SendString("#ERR bad vision command arguments\r\n");
    return 1U;
}

static void TuningConsole_ExecuteLine(char *line)
{
    char *tokens[TUNING_TOKEN_MAX];
    uint8_t tokenCount;
    int32_t left;
    int32_t right;
    int32_t leftDistance;
    int32_t rightDistance;

    TuningConsole_ToLower(line);
    tokenCount = TuningConsole_Split(line, tokens, TUNING_TOKEN_MAX);
    if (tokenCount == 0U) {
        return;
    }

    if ((TuningConsole_TextEquals(tokens[0], "mode") != 0U) &&
        (tokenCount == 2U)) {
        if (TuningConsole_TextEquals(tokens[1], "chassis") != 0U) {
            TuningConsole_SetMode(TUNING_MODE_CHASSIS);
        } else if (TuningConsole_TextEquals(tokens[1], "gimbal") != 0U) {
            TuningConsole_SetMode(TUNING_MODE_GIMBAL);
        } else if (TuningConsole_TextEquals(tokens[1], "vision") != 0U) {
            TuningConsole_SetMode(TUNING_MODE_VISION);
        } else {
            LogUart_SendString(
                "#ERR mode expects chassis, gimbal, or vision\r\n");
        }
        return;
    }
    if (TuningConsole_ExecuteGimbalCommand(tokens, tokenCount) != 0U) {
        return;
    }
    if (TuningConsole_ExecuteVisionCommand(tokens, tokenCount) != 0U) {
        return;
    }
    if ((g_tuningMode != TUNING_MODE_CHASSIS) &&
        (TuningConsole_TextEquals(tokens[0], "help") == 0U) &&
        (TuningConsole_TextEquals(tokens[0], "show") == 0U) &&
        (TuningConsole_TextEquals(tokens[0], "oled") == 0U) &&
        (TuningConsole_TextEquals(tokens[0], "stop") == 0U)) {
        LogUart_SendString(
            "#ERR chassis command blocked; send mode chassis first\r\n");
        return;
    }

    if ((TuningConsole_TextEquals(tokens[0], "help") != 0U) &&
        (tokenCount == 1U)) {
        TuningConsole_SendHelp();
    } else if ((TuningConsole_TextEquals(tokens[0], "show") != 0U) &&
        (tokenCount == 1U)) {
        TuningConsole_SendDetails();
    } else if ((TuningConsole_TextEquals(tokens[0], "oled") != 0U) &&
        (tokenCount == 2U)) {
        TuningConsole_SelectOledPage(tokens[1]);
    } else if ((TuningConsole_TextEquals(tokens[0], "gray") != 0U) &&
        ((tokenCount == 1U) || (tokenCount == 2U))) {
        if ((tokenCount == 1U) ||
            (TuningConsole_TextEquals(tokens[1], "on") != 0U)) {
            TuningConsole_StartGrayTest();
        } else if (TuningConsole_TextEquals(tokens[1], "off") != 0U) {
            TuningConsole_StopGrayTest();
            g_oledPage = TUNING_CONSOLE_OLED_FF;
            TuningConsole_ResetAverage();
            LogUart_SendString("#OK gray off; PWM=0\r\n");
            g_forceStatus = 1U;
        } else {
            LogUart_SendString("#ERR gray expects on or off\r\n");
        }
    } else if ((TuningConsole_TextEquals(tokens[0], "stop") != 0U) &&
        (tokenCount == 1U)) {
        TuningConsole_CancelSetSampling();
        TuningConsole_CancelEncoderMove();
        TuningConsole_StopGrayTest();
        EncoderMotor_SetOpenLoopPwm(0, 0);
        GimbalAttitude_Stop();
        TuningConsole_StopVisionActivity();
        TuningConsole_ResetAverage();
        LogUart_SendString("#OK stop\r\n");
        g_forceStatus = 1U;
    } else if ((TuningConsole_TextEquals(tokens[0], "clear") != 0U) &&
        (tokenCount == 1U)) {
        EncoderMotor_ClearIntegral();
        LogUart_SendString("#OK integral cleared\r\n");
        g_forceStatus = 1U;
    } else if ((TuningConsole_TextEquals(tokens[0], "avg") != 0U) &&
        (tokenCount == 1U)) {
        TuningConsole_ResetAverage();
        LogUart_SendString("#OK average reset\r\n");
        g_forceStatus = 1U;
    } else if ((TuningConsole_TextEquals(tokens[0], "set") != 0U) &&
        (tokenCount == 2U) &&
        (TuningConsole_TextEquals(tokens[1], "clear") != 0U)) {
        TuningConsole_CancelSetSampling();
        TuningConsole_CancelEncoderMove();
        TuningConsole_ClearSetReferences();
        TuningConsole_StopGrayTest();
        EncoderMotor_SetOpenLoopPwm(0, 0);
        TuningConsole_ResetAverage();
        LogUart_SendString("#OK set points cleared; PWM=0\r\n");
        g_forceStatus = 1U;
    } else if ((TuningConsole_TextEquals(tokens[0], "set") != 0U) &&
        (tokenCount == 3U)) {
        if (TuningConsole_ParsePair(tokens, &left, &right) != 0U) {
            TuningConsole_StartSetSampling(left, right);
        }
    } else if ((TuningConsole_TextEquals(tokens[0], "pwm") != 0U) &&
        (tokenCount == 3U)) {
        if (TuningConsole_ParsePair(tokens, &left, &right) == 0U) {
            return;
        }
        if ((left < -TUNING_PWM_PERCENT_MAX) ||
            (left > TUNING_PWM_PERCENT_MAX) ||
            (right < -TUNING_PWM_PERCENT_MAX) ||
            (right > TUNING_PWM_PERCENT_MAX)) {
            LogUart_SendString("#ERR pwm range is -100..100 percent\r\n");
            return;
        }
        TuningConsole_CancelSetSampling();
        TuningConsole_CancelEncoderMove();
        TuningConsole_StopGrayTest();
        EncoderMotor_SetOpenLoopPwm(TuningConsole_PercentToPwm(left),
            TuningConsole_PercentToPwm(right));
        TuningConsole_ResetAverage();
        TuningConsole_SendPair("pwm_pct", left, right);
        TuningConsole_SendPair("pwm_raw",
            TuningConsole_PercentToPwm(left),
            TuningConsole_PercentToPwm(right));
        g_forceStatus = 1U;
    } else if ((TuningConsole_TextEquals(tokens[0], "target") != 0U) &&
        (tokenCount == 3U)) {
        if (TuningConsole_ParsePair(tokens, &left, &right) == 0U) {
            return;
        }
        if ((left < -TUNING_TARGET_COUNT_LIMIT) ||
            (left > TUNING_TARGET_COUNT_LIMIT) ||
            (right < -TUNING_TARGET_COUNT_LIMIT) ||
            (right > TUNING_TARGET_COUNT_LIMIT)) {
            LogUart_SendString(
                "#ERR target range is -100..100 count/20ms\r\n");
            return;
        }
        TuningConsole_CancelSetSampling();
        TuningConsole_CancelEncoderMove();
        TuningConsole_StopGrayTest();
        EncoderMotor_SetCalibrationTargets((int16_t)left, (int16_t)right);
        TuningConsole_ResetAverage();
        TuningConsole_SendPair("target_count", left, right);
        g_forceStatus = 1U;
    } else if ((TuningConsole_TextEquals(tokens[0], "move") != 0U) &&
        (tokenCount == 5U)) {
        if ((TuningConsole_ParseInt32(tokens[1], &left) != 0U) &&
            (TuningConsole_ParseInt32(tokens[2], &right) != 0U) &&
            (TuningConsole_ParseInt32(tokens[3], &leftDistance) != 0U) &&
            (TuningConsole_ParseInt32(tokens[4], &rightDistance) != 0U)) {
            TuningConsole_StartEncoderMove(left, right,
                leftDistance, rightDistance);
        } else {
            LogUart_SendString("#ERR move expects four integers\r\n");
        }
    } else if (((TuningConsole_TextEquals(tokens[0], "ff") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "kp") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "ki") != 0U)) &&
        (tokenCount == 3U)) {
        if (TuningConsole_ParsePair(tokens, &left, &right) != 0U) {
            TuningConsole_ApplyGain(tokens[0], left, right);
        }
    } else if (((TuningConsole_TextEquals(tokens[0], "start") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "runstart") != 0U) ||
        (TuningConsole_TextEquals(tokens[0], "ilim") != 0U)) &&
        (tokenCount == 3U)) {
        if (TuningConsole_ParsePair(tokens, &left, &right) != 0U) {
            TuningConsole_ApplyPwmParameter(tokens[0], left, right);
        }
    } else if ((TuningConsole_TextEquals(tokens[0], "ffcalc") != 0U) &&
        (tokenCount == 5U)) {
        TuningConsole_CalculateFf(tokens);
    } else {
        LogUart_SendString("#ERR bad command, type help\r\n");
    }
}

static void TuningConsole_ProcessRx(void)
{
    uint8_t data;

    while (LogUart_TryReadByte(&data) != 0U) {
        if ((data == '\r') || (data == '\n')) {
            if (g_discardLine != 0U) {
                g_discardLine = 0U;
                g_lineLength = 0U;
                LogUart_SendString("#ERR command line too long\r\n");
            } else if (g_lineLength != 0U) {
                g_line[g_lineLength] = '\0';
                TuningConsole_ExecuteLine(g_line);
                g_lineLength = 0U;
            }
        } else if ((data == 8U) || (data == 127U)) {
            if ((g_discardLine == 0U) && (g_lineLength > 0U)) {
                --g_lineLength;
            }
        } else if (g_discardLine == 0U) {
            if (g_lineLength < (TUNING_LINE_MAX - 1U)) {
                g_line[g_lineLength++] = (char)data;
            } else {
                g_discardLine = 1U;
            }
        }
    }
}

void TuningConsole_Start(void)
{
    if (g_active != 0U) {
        return;
    }
    g_lineLength = 0U;
    g_discardLine = 0U;
    g_active = 1U;
    g_forceStatus = 0U;
    g_oledPage = TUNING_CONSOLE_OLED_FF;
    g_tuningMode = TUNING_MODE_CHASSIS;
    g_gimbalPlotEnabled = 0U;
    g_visionSource = STATICCONFIG_MODE_CENTER;
    g_visionPlotEnabled = 0U;
    g_grayActive = 0U;
    g_grayMask = 0U;
    TuningConsole_CancelEncoderMove();
    TuningConsole_CancelSetSampling();
    TuningConsole_ClearSetReferences();
    LogUart_ClearRx();
    GimbalAttitude_Stop();
    TuningConsole_StopVisionActivity();
    EncoderMotor_EnterCalibration();
    TuningConsole_ResetAverage();
    g_lastStatusTime = xTaskGetTickCount();
    LogUart_SendString(
        "# Task5 PID calibration ready; PWM commands use percent\r\n");
    TuningConsole_SendHelp();
    TuningConsole_SendStatus();
}

void TuningConsole_Stop(void)
{
    if (g_active == 0U) {
        return;
    }
    g_grayActive = 0U;
    g_grayMask = 0U;
    TuningConsole_CancelEncoderMove();
    GimbalAttitude_Stop();
    TuningConsole_StopVisionActivity();
    EncoderMotor_ExitCalibration();
    TuningConsole_CancelSetSampling();
    TuningConsole_ClearSetReferences();
    g_active = 0U;
    g_lineLength = 0U;
    g_discardLine = 0U;
    g_forceStatus = 0U;
    g_gimbalPlotEnabled = 0U;
    g_visionPlotEnabled = 0U;
    LogUart_SendString("# Task5 calibration stopped; PWM=0\r\n");
}

void TuningConsole_Task(void)
{
    TickType_t now;

    if (g_active == 0U) {
        return;
    }
    TuningConsole_ProcessRx();
    if (g_tuningMode == TUNING_MODE_CHASSIS) {
        TuningConsole_UpdateAverage();
        TuningConsole_UpdateEncoderMove();
    }
    now = xTaskGetTickCount();
    if ((g_tuningMode == TUNING_MODE_GIMBAL) &&
        (g_gimbalPlotEnabled != 0U) &&
        ((now - g_lastGimbalPlotTime) >=
            pdMS_TO_TICKS(TUNING_GIMBAL_PLOT_PERIOD_MS))) {
        g_lastGimbalPlotTime = now;
        TuningConsole_SendGimbalPlotFrame();
    }
    if ((g_tuningMode == TUNING_MODE_VISION) &&
        (g_visionPlotEnabled != 0U) &&
        ((now - g_lastVisionPlotTime) >=
            pdMS_TO_TICKS(TUNING_VISION_PLOT_PERIOD_MS))) {
        g_lastVisionPlotTime = now;
        TuningConsole_SendVisionPlotFrame();
    }
    if (g_tuningMode == TUNING_MODE_CHASSIS) {
        TuningConsole_UpdateSetSampling(now);
    }
    if ((g_forceStatus != 0U) ||
        ((now - g_lastStatusTime) >=
            pdMS_TO_TICKS(TUNING_STATUS_PERIOD_MS))) {
        g_forceStatus = 0U;
        g_lastStatusTime = now;
        TuningConsole_SendStatus();
    }
}

void TuningConsole_ChassisControlPeriod(void)
{
    uint8_t mask;

    if ((g_active == 0U) || (g_tuningMode != TUNING_MODE_CHASSIS) ||
        (g_grayActive == 0U)) {
        return;
    }
    mask = Gray_ReadDigitalMaskFast();
    g_grayMask = mask;
    (void)MotorNoYaw_ApplyTask1LineCommand(mask);
}

uint8_t TuningConsole_IsActive(void)
{
    return g_active;
}

void TuningConsole_GetDisplayStatus(TuningConsoleDisplayStatus *status)
{
    EncoderMotorSnapshot snapshot;
    GimbalAttitudeSnapshot gimbal;

    if (status == 0) {
        return;
    }

    EncoderMotor_GetSnapshot(&snapshot);
    GimbalAttitude_GetSnapshot(&gimbal);
    status->oledPage = g_oledPage;
    status->gimbalMode = (g_tuningMode == TUNING_MODE_GIMBAL) ? 1U : 0U;
    status->visionMode = (g_tuningMode == TUNING_MODE_VISION) ? 1U : 0U;
    status->visionHasFrame = Vision_HasFrame();
    status->visionFrameCount = Vision_GetFrameCount();
    status->visionRawX = Vision_GetRawX();
    status->visionRawY = Vision_GetRawY();
    status->visionStageScaleX10 = Vision_GetStageScaleX10();
    status->visionCommandX = Gimbal_GetCommandX();
    status->visionCommandY = Gimbal_GetCommandY();
    status->visionYawGainQ1024 = Vision_GetYawGainQ1024();
    status->gimbalState = (uint8_t)gimbal.motion.state;
    status->gimbalHoldEnabled = gimbal.holdEnabled;
    status->gimbalFeedForwardEnabled = gimbal.feedForwardEnabled;
    status->gimbalFeedbackFresh = gimbal.feedbackFresh;
    status->gimbalFeedForwardFresh = gimbal.feedForwardFresh;
    status->gimbalCalibrationCount = gimbal.motion.calibrationCount;
    status->gimbalCalibrationTarget = gimbal.motion.calibrationTarget;
    status->gimbalYawX100 = gimbal.feedbackYawX100;
    status->gimbalRateX100PerSec = gimbal.feedbackRateX100PerSec;
    status->gimbalAngleErrorX100 = gimbal.angleErrorX100;
    status->gimbalCommandSps = gimbal.commandSps;
    if (g_setState == TUNING_SET_SETTLING) {
        status->setStage = TUNING_CONSOLE_SET_STAGE_SETTLING;
    } else if (g_setState == TUNING_SET_SAMPLING) {
        status->setStage = TUNING_CONSOLE_SET_STAGE_SAMPLING;
    } else {
        status->setStage = TUNING_CONSOLE_SET_STAGE_IDLE;
    }
    status->leftResult = g_setResult[ENCODER_MOTOR_LEFT];
    status->rightResult = g_setResult[ENCODER_MOTOR_RIGHT];
    status->sampleCount = g_averageSampleCount;
    status->sampleTarget = TUNING_SET_SAMPLE_WINDOWS;
    status->leftFfQ1024 =
        snapshot.tuning[ENCODER_MOTOR_LEFT].ffQ1024;
    status->rightFfQ1024 =
        snapshot.tuning[ENCODER_MOTOR_RIGHT].ffQ1024;
    status->leftStartPercent = TuningConsole_PwmToPercent(
        snapshot.tuning[ENCODER_MOTOR_LEFT].startPwm);
    status->rightStartPercent = TuningConsole_PwmToPercent(
        snapshot.tuning[ENCODER_MOTOR_RIGHT].startPwm);
    status->leftRunStartPercent = TuningConsole_PwmToPercent(
        snapshot.tuning[ENCODER_MOTOR_LEFT].runStartPwm);
    status->rightRunStartPercent = TuningConsole_PwmToPercent(
        snapshot.tuning[ENCODER_MOTOR_RIGHT].runStartPwm);
    status->leftPwmPercent = TuningConsole_PwmToPercent(
        snapshot.outputPwm[ENCODER_MOTOR_LEFT]);
    status->rightPwmPercent = TuningConsole_PwmToPercent(
        snapshot.outputPwm[ENCODER_MOTOR_RIGHT]);
    status->leftTargetCounts =
        snapshot.targetCounts[ENCODER_MOTOR_LEFT];
    status->rightTargetCounts =
        snapshot.targetCounts[ENCODER_MOTOR_RIGHT];
    status->leftFeedbackCounts =
        snapshot.feedbackCounts[ENCODER_MOTOR_LEFT];
    status->rightFeedbackCounts =
        snapshot.feedbackCounts[ENCODER_MOTOR_RIGHT];
    status->leftAverageCounts =
        TuningConsole_GetAverage(ENCODER_MOTOR_LEFT);
    status->rightAverageCounts =
        TuningConsole_GetAverage(ENCODER_MOTOR_RIGHT);
    status->grayMask = g_grayMask;
}

#endif /* CAR_PROFILE_IS_FULL */
