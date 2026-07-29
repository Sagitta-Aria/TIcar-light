#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "tuning_console.h"

#include "FreeRTOS.h"
#include "task.h"

#include "control_config.h"
#include "encoder_motor.h"
#include "gmr_yaw_control.h"
#include "h7_gyro_link.h"
#include "log_uart.h"
#include "tuning_common.h"

#define GMR_TUNING_LINE_MAX                 (80U)
#define GMR_TUNING_TOKEN_MAX                (5U)
#define GMR_TUNING_STATUS_PERIOD_MS         (500U)
#define GMR_TUNING_SET_SETTLE_MS            (4000U)
#define GMR_TUNING_SET_SAMPLE_DURATION_MS   (1000U)
#define GMR_TUNING_SET_SAMPLE_WINDOWS \
    (GMR_TUNING_SET_SAMPLE_DURATION_MS / CHASSIS_CONTROL_PERIOD_MS)

typedef struct {
    int16_t pwm;
    int64_t feedbackSum;
    uint32_t sampleCount;
    uint8_t valid;
} GmrSetPoint;

static char g_line[GMR_TUNING_LINE_MAX];
static uint8_t g_lineLength;
static uint8_t g_discardLine;
static uint8_t g_active;
static uint8_t g_forceStatus;
static uint8_t g_skipNextSample;
static TickType_t g_lastStatusTick;
static uint32_t g_lastSampleSequence;
static int64_t g_feedbackSum[ENCODER_MOTOR_COUNT];
static uint32_t g_averageSampleCount;
static TuningConsoleOledPage g_oledPage;
static TuningConsoleSetStage g_setStage;
static TickType_t g_setStartTick;
static int16_t g_setPwm[ENCODER_MOTOR_COUNT];
static GmrSetPoint g_setReference[ENCODER_MOTOR_COUNT];
static TuningConsoleSetResult g_setResult[ENCODER_MOTOR_COUNT];

#define TextEquals   TuningCommon_TextEquals
#define ToLower      TuningCommon_ToLower
#define ParseInt32   TuningCommon_ParseInt32
#define Split        TuningCommon_Split
#define PercentToPwm TuningCommon_PercentToPwm
#define PwmToPercent TuningCommon_PwmToPercent
#define GMR_TUNING_PWM_PERCENT_MAX TUNING_COMMON_PWM_PERCENT_MAX

static uint8_t ParsePair(char *tokens[], int32_t *left, int32_t *right)
{
    if ((ParseInt32(tokens[1], left) == 0U) ||
        (ParseInt32(tokens[2], right) == 0U)) {
        LogUart_SendString("#ERR expected two integers\r\n");
        return 0U;
    }
    return 1U;
}

static int32_t GetAverage(uint8_t index)
{
    if ((index >= ENCODER_MOTOR_COUNT) || (g_averageSampleCount == 0U)) {
        return 0;
    }
    return (int32_t)(g_feedbackSum[index] / (int64_t)g_averageSampleCount);
}

static void ResetAverage(void)
{
    EncoderMotorSnapshot snapshot;

    EncoderMotor_GetSnapshot(&snapshot);
    g_feedbackSum[ENCODER_MOTOR_LEFT] = 0;
    g_feedbackSum[ENCODER_MOTOR_RIGHT] = 0;
    g_averageSampleCount = 0U;
    g_lastSampleSequence = snapshot.sampleSequence;
    g_skipNextSample = 1U;
}

static void UpdateAverage(void)
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

static void CancelSet(void)
{
    g_setStage = TUNING_CONSOLE_SET_STAGE_IDLE;
    g_setPwm[ENCODER_MOTOR_LEFT] = 0;
    g_setPwm[ENCODER_MOTOR_RIGHT] = 0;
}

static void ClearSetReferences(void)
{
    uint8_t i;

    for (i = 0U; i < ENCODER_MOTOR_COUNT; ++i) {
        g_setReference[i].valid = 0U;
        g_setResult[i] = TUNING_CONSOLE_SET_RESULT_NONE;
    }
}

static void SendPair(const char *name, int32_t left, int32_t right)
{
    LogUart_SendString("# ");
    LogUart_SendString(name);
    LogUart_SendString("=");
    LogUart_SendSigned(left);
    LogUart_SendString(",");
    LogUart_SendSigned(right);
    LogUart_SendString("\r\n");
}

static void SendPlotFrame(const EncoderMotorSnapshot *snapshot)
{
    LogUart_SendString("P ");
    LogUart_SendSigned(snapshot->targetCounts[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot->feedbackCounts[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendSigned(PwmToPercent(snapshot->outputPwm[ENCODER_MOTOR_LEFT]));
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot->targetCounts[ENCODER_MOTOR_RIGHT]);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot->feedbackCounts[ENCODER_MOTOR_RIGHT]);
    LogUart_SendString(",");
    LogUart_SendSigned(PwmToPercent(snapshot->outputPwm[ENCODER_MOTOR_RIGHT]));
    LogUart_SendString(",");
    LogUart_SendUnsigned(snapshot->pwmCompareCounts[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot->tuning[ENCODER_MOTOR_LEFT].ffQ1024);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot->tuning[ENCODER_MOTOR_RIGHT].ffQ1024);
    LogUart_SendString("\r\n");
}

static void SendYawFrame(void)
{
    GmrYawControlSnapshot yaw;

    GmrYawControl_GetSnapshot(&yaw);
    LogUart_SendString("Y ");
    LogUart_SendSigned(yaw.yawX100);
    LogUart_SendString(",");
    LogUart_SendSigned(yaw.targetYawX100);
    LogUart_SendString(",");
    LogUart_SendSigned(yaw.errorX100);
    LogUart_SendString(",");
    LogUart_SendSigned(yaw.wheelCommandCounts);
    LogUart_SendString(",");
    LogUart_SendSigned(yaw.leftTargetCounts);
    LogUart_SendString(",");
    LogUart_SendSigned(yaw.rightTargetCounts);
    LogUart_SendString(",");
    LogUart_SendSigned(yaw.yawRateX100PerSec);
    LogUart_SendString(",");
    LogUart_SendUnsigned(yaw.sensorAgeMs);
    LogUart_SendString(",");
    LogUart_SendUnsigned(yaw.active);
    LogUart_SendString(",");
    LogUart_SendUnsigned(yaw.sensorFresh);
    LogUart_SendString("\r\n");
}

static void SendH7ImuFrame(void)
{
    H7GyroLinkFeedback h7;
    TickType_t now = xTaskGetTickCount();
    uint32_t ageMs;
    uint8_t fresh;

#if CAR_LIBRARY_H7_IMU_ENABLED
    (void)H7GyroLink_GetFeedback(&h7);
    ageMs = (h7.angleFrameCount == 0U) ? 0xFFFFFFFFUL :
        (uint32_t)(now - h7.angleFrameTick);
    fresh = (uint8_t)((h7.angleFrameCount != 0U) &&
        (ageMs <= (uint32_t)GMR_H7_IMU_STALE_MS));
#else
    (void)now;
    h7.rollX100 = 0;
    h7.pitchX100 = 0;
    h7.yawUnwrappedX100 = 0;
    h7.yawRateX100PerSec = 0;
    ageMs = 0xFFFFFFFFUL;
    fresh = 0U;
#endif
    LogUart_SendString("H ");
    LogUart_SendSigned(h7.rollX100);
    LogUart_SendString(",");
    LogUart_SendSigned(h7.pitchX100);
    LogUart_SendString(",");
    LogUart_SendSigned(h7.yawUnwrappedX100);
    LogUart_SendString(",");
    LogUart_SendSigned(h7.yawRateX100PerSec);
    LogUart_SendString(",");
    LogUart_SendUnsigned(ageMs);
    LogUart_SendString(",");
    LogUart_SendUnsigned(fresh);
    LogUart_SendString("\r\n");
}

static const char *ModeName(EncoderMotorMode mode)
{
    if (mode == ENCODER_MOTOR_MODE_CALIBRATION_OPEN_LOOP) {
        return "open";
    }
    if (mode == ENCODER_MOTOR_MODE_CALIBRATION_CLOSED_LOOP) {
        return "closed";
    }
    return "normal";
}

static void SendStatus(void)
{
    EncoderMotorSnapshot snapshot;

    EncoderMotor_GetSnapshot(&snapshot);
    LogUart_SendString("D mode=");
    LogUart_SendString(ModeName(snapshot.mode));
    LogUart_SendString(" set=");
    LogUart_SendUnsigned((uint32_t)g_setStage);
    LogUart_SendString(" pwm_pct=");
    LogUart_SendSigned(PwmToPercent(snapshot.outputPwm[ENCODER_MOTOR_LEFT]));
    LogUart_SendString(",");
    LogUart_SendSigned(PwmToPercent(snapshot.outputPwm[ENCODER_MOTOR_RIGHT]));
    LogUart_SendString(" target=");
    LogUart_SendSigned(snapshot.targetCounts[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot.targetCounts[ENCODER_MOTOR_RIGHT]);
    LogUart_SendString(" count=");
    LogUart_SendSigned(snapshot.feedbackCounts[ENCODER_MOTOR_LEFT]);
    LogUart_SendString(",");
    LogUart_SendSigned(snapshot.feedbackCounts[ENCODER_MOTOR_RIGHT]);
    LogUart_SendString(" avg=");
    LogUart_SendSigned(GetAverage(ENCODER_MOTOR_LEFT));
    LogUart_SendString(",");
    LogUart_SendSigned(GetAverage(ENCODER_MOTOR_RIGHT));
    LogUart_SendString(" n=");
    LogUart_SendUnsigned(g_averageSampleCount);
    LogUart_SendString("\r\n");
    SendPlotFrame(&snapshot);
    SendYawFrame();
    SendH7ImuFrame();
}

static void SendDetails(void)
{
    EncoderMotorSnapshot s;

    EncoderMotor_GetSnapshot(&s);
    SendStatus();
    SendPair("start_raw", s.tuning[ENCODER_MOTOR_LEFT].startPwm,
        s.tuning[ENCODER_MOTOR_RIGHT].startPwm);
    SendPair("runstart_raw", s.tuning[ENCODER_MOTOR_LEFT].runStartPwm,
        s.tuning[ENCODER_MOTOR_RIGHT].runStartPwm);
    SendPair("ff_q1024", s.tuning[ENCODER_MOTOR_LEFT].ffQ1024,
        s.tuning[ENCODER_MOTOR_RIGHT].ffQ1024);
    SendPair("kp_q1024", s.tuning[ENCODER_MOTOR_LEFT].kpQ1024,
        s.tuning[ENCODER_MOTOR_RIGHT].kpQ1024);
    SendPair("ki_q1024", s.tuning[ENCODER_MOTOR_LEFT].kiQ1024,
        s.tuning[ENCODER_MOTOR_RIGHT].kiQ1024);
    SendPair("ilim_raw", s.tuning[ENCODER_MOTOR_LEFT].integralLimitPwm,
        s.tuning[ENCODER_MOTOR_RIGHT].integralLimitPwm);
}

static void SendHelp(void)
{
    LogUart_SendString("# GMR Task2 chassis PID console\r\n");
    LogUart_SendString("# pwm L R | target L R | set L R | set clear\r\n");
    LogUart_SendString("# start/runstart/ilim L R use percent\r\n");
    LogUart_SendString("# ff/kp/ki L R use Q1024 | ffcalc P1 C1 P2 C2\r\n");
    LogUart_SendString(
        "# Task5 auto-holds M0 startup yaw at base speed; deadband=10deg\r\n");
    LogUart_SendString(
        "# angle DEG shifts target relative to current yaw, range -180..180\r\n");
    LogUart_SendString("# oled ff|start|speed|pid|yaw | avg | clear | stop | show\r\n");
}

static void ApplyTuning(const EncoderMotorTuning *left,
    const EncoderMotorTuning *right)
{
    EncoderMotor_SetTuning(left, right);
    EncoderMotor_ClearIntegral();
    g_forceStatus = 1U;
}

static void ApplyGain(const char *cmd, int32_t left, int32_t right)
{
    EncoderMotorSnapshot s;

    if ((left < 0) || (right < 0)) {
        LogUart_SendString("#ERR gains must be >=0\r\n");
        return;
    }
    EncoderMotor_GetSnapshot(&s);
    if (TextEquals(cmd, "ff") != 0U) {
        s.tuning[ENCODER_MOTOR_LEFT].ffQ1024 = left;
        s.tuning[ENCODER_MOTOR_RIGHT].ffQ1024 = right;
    } else if (TextEquals(cmd, "kp") != 0U) {
        s.tuning[ENCODER_MOTOR_LEFT].kpQ1024 = left;
        s.tuning[ENCODER_MOTOR_RIGHT].kpQ1024 = right;
    } else {
        s.tuning[ENCODER_MOTOR_LEFT].kiQ1024 = left;
        s.tuning[ENCODER_MOTOR_RIGHT].kiQ1024 = right;
    }
    ApplyTuning(&s.tuning[ENCODER_MOTOR_LEFT],
        &s.tuning[ENCODER_MOTOR_RIGHT]);
    SendPair(cmd, left, right);
}

static void ApplyPwmParameter(const char *cmd, int32_t left, int32_t right)
{
    EncoderMotorSnapshot s;
    int16_t leftRaw;
    int16_t rightRaw;

    if ((left < 0) || (left > GMR_TUNING_PWM_PERCENT_MAX) ||
        (right < 0) || (right > GMR_TUNING_PWM_PERCENT_MAX)) {
        LogUart_SendString("#ERR percent range is 0..100\r\n");
        return;
    }
    leftRaw = PercentToPwm(left);
    rightRaw = PercentToPwm(right);
    EncoderMotor_GetSnapshot(&s);
    if (TextEquals(cmd, "start") != 0U) {
        s.tuning[ENCODER_MOTOR_LEFT].startPwm = leftRaw;
        s.tuning[ENCODER_MOTOR_RIGHT].startPwm = rightRaw;
    } else if (TextEquals(cmd, "runstart") != 0U) {
        s.tuning[ENCODER_MOTOR_LEFT].runStartPwm = leftRaw;
        s.tuning[ENCODER_MOTOR_RIGHT].runStartPwm = rightRaw;
    } else {
        s.tuning[ENCODER_MOTOR_LEFT].integralLimitPwm = leftRaw;
        s.tuning[ENCODER_MOTOR_RIGHT].integralLimitPwm = rightRaw;
    }
    ApplyTuning(&s.tuning[ENCODER_MOTOR_LEFT],
        &s.tuning[ENCODER_MOTOR_RIGHT]);
    SendPair(cmd, left, right);
    SendPair("raw", leftRaw, rightRaw);
}

static void CalculateFf(int32_t pwm1Percent, int32_t count1,
    int32_t pwm2Percent, int32_t count2)
{
    int16_t pwm1Raw;
    int16_t pwm2Raw;
    int64_t numerator;
    int64_t denominator;
    int64_t ff;
    int64_t interceptNumerator;
    int64_t runStart;

    if ((pwm1Percent < -GMR_TUNING_PWM_PERCENT_MAX) ||
        (pwm1Percent > GMR_TUNING_PWM_PERCENT_MAX) ||
        (pwm2Percent < -GMR_TUNING_PWM_PERCENT_MAX) ||
        (pwm2Percent > GMR_TUNING_PWM_PERCENT_MAX) ||
        (pwm1Percent == pwm2Percent) || (count1 == count2)) {
        LogUart_SendString("#ERR ffcalc expects two different PWM/count points\r\n");
        return;
    }
    pwm1Raw = PercentToPwm(pwm1Percent);
    pwm2Raw = PercentToPwm(pwm2Percent);
    if (((pwm1Raw > 0) && (count1 <= 0)) ||
        ((pwm1Raw < 0) && (count1 >= 0)) ||
        ((pwm2Raw > 0) && (count2 <= 0)) ||
        ((pwm2Raw < 0) && (count2 >= 0)) ||
        (((int32_t)pwm1Raw * pwm2Raw) < 0)) {
        LogUart_SendString("#ERR ffcalc points must use one direction and matching count sign\r\n");
        return;
    }
    numerator = ((int64_t)pwm2Raw - pwm1Raw) * CHASSIS_Q1024_SCALE;
    denominator = (int64_t)count2 - count1;
    if (denominator < 0) {
        denominator = -denominator;
        numerator = -numerator;
    }
    if (numerator <= 0) {
        LogUart_SendString("#ERR ffcalc slope must be positive\r\n");
        return;
    }
    ff = (numerator + denominator / 2) / denominator;
    interceptNumerator = (int64_t)pwm1Raw * CHASSIS_Q1024_SCALE -
        ff * count1;
    if (interceptNumerator < 0) {
        interceptNumerator = -interceptNumerator;
    }
    runStart = (interceptNumerator + CHASSIS_Q1024_SCALE / 2) /
        CHASSIS_Q1024_SCALE;
    if ((ff > 0x7FFFFFFFLL) || (runStart > CHASSIS_PWM_LIMIT_COUNTS)) {
        LogUart_SendString("#ERR ffcalc result exceeds limits\r\n");
        return;
    }
    LogUart_SendString("#FFCALC ff_q1024=");
    LogUart_SendSigned((int32_t)ff);
    LogUart_SendString(" runstart_raw=");
    LogUart_SendSigned((int32_t)runStart);
    LogUart_SendString(" runstart_pct=");
    LogUart_SendSigned(PwmToPercent((int32_t)runStart));
    LogUart_SendString("\r\n");
}

static uint8_t ApplySetPoint(uint8_t index, const GmrSetPoint *point,
    EncoderMotorSnapshot *snapshot)
{
    GmrSetPoint *reference = &g_setReference[index];
    const char *name = (index == ENCODER_MOTOR_LEFT) ? "left" : "right";
    int64_t numerator;
    int64_t denominator;
    int64_t ff;
    int64_t interceptNumerator;
    int64_t interceptDenominator;
    int64_t runStart;

    if (((point->pwm > 0) && (point->feedbackSum <= 0)) ||
        ((point->pwm < 0) && (point->feedbackSum >= 0))) {
        g_setResult[index] = TUNING_CONSOLE_SET_RESULT_ERROR;
        LogUart_SendString("#ERR set count sign mismatch ");
        LogUart_SendString(name);
        LogUart_SendString("\r\n");
        return 0U;
    }
    if (reference->valid == 0U) {
        *reference = *point;
        reference->valid = 1U;
        g_setResult[index] = TUNING_CONSOLE_SET_RESULT_POINT_STORED;
        LogUart_SendString("#SET point1 stored ");
        LogUart_SendString(name);
        LogUart_SendString("\r\n");
        return 0U;
    }
    if ((reference->pwm == point->pwm) ||
        (((int32_t)reference->pwm * point->pwm) < 0)) {
        g_setResult[index] = TUNING_CONSOLE_SET_RESULT_ERROR;
        LogUart_SendString("#ERR set needs two same-direction different PWM points\r\n");
        return 0U;
    }
    numerator = ((int64_t)point->pwm - reference->pwm) *
        CHASSIS_Q1024_SCALE * reference->sampleCount * point->sampleCount;
    denominator = point->feedbackSum * reference->sampleCount -
        reference->feedbackSum * point->sampleCount;
    if (denominator == 0) {
        g_setResult[index] = TUNING_CONSOLE_SET_RESULT_ERROR;
        LogUart_SendString("#ERR set average did not change\r\n");
        return 0U;
    }
    if (denominator < 0) {
        denominator = -denominator;
        numerator = -numerator;
    }
    if (numerator <= 0) {
        g_setResult[index] = TUNING_CONSOLE_SET_RESULT_ERROR;
        LogUart_SendString("#ERR set slope must be positive\r\n");
        return 0U;
    }
    ff = (numerator + denominator / 2) / denominator;
    interceptNumerator =
        (int64_t)reference->pwm * CHASSIS_Q1024_SCALE *
        reference->sampleCount - ff * reference->feedbackSum;
    if (interceptNumerator < 0) {
        interceptNumerator = -interceptNumerator;
    }
    interceptDenominator =
        (int64_t)CHASSIS_Q1024_SCALE * reference->sampleCount;
    runStart = (interceptNumerator + interceptDenominator / 2) /
        interceptDenominator;
    if ((ff > 0x7FFFFFFFLL) || (runStart > CHASSIS_PWM_LIMIT_COUNTS)) {
        g_setResult[index] = TUNING_CONSOLE_SET_RESULT_ERROR;
        LogUart_SendString("#ERR set fit exceeds limits\r\n");
        return 0U;
    }
    snapshot->tuning[index].ffQ1024 = (int32_t)ff;
    snapshot->tuning[index].runStartPwm = (int16_t)runStart;
    g_setResult[index] = TUNING_CONSOLE_SET_RESULT_FF_APPLIED;
    *reference = *point;
    LogUart_SendString("#SET ");
    LogUart_SendString(name);
    LogUart_SendString(" ff=");
    LogUart_SendSigned((int32_t)ff);
    LogUart_SendString(" runstart_raw=");
    LogUart_SendSigned((int32_t)runStart);
    LogUart_SendString("\r\n");
    return 1U;
}

static void FinishSet(void)
{
    EncoderMotorSnapshot snapshot;
    GmrSetPoint point;
    uint8_t i;
    uint8_t apply = 0U;

    EncoderMotor_SetOpenLoopPwm(0, 0);
    EncoderMotor_GetSnapshot(&snapshot);
    for (i = 0U; i < ENCODER_MOTOR_COUNT; ++i) {
        if (g_setPwm[i] == 0) {
            continue;
        }
        point.pwm = g_setPwm[i];
        point.feedbackSum = g_feedbackSum[i];
        point.sampleCount = g_averageSampleCount;
        point.valid = 1U;
        if (ApplySetPoint(i, &point, &snapshot) != 0U) {
            apply = 1U;
        }
    }
    if (apply != 0U) {
        ApplyTuning(&snapshot.tuning[ENCODER_MOTOR_LEFT],
            &snapshot.tuning[ENCODER_MOTOR_RIGHT]);
    }
    CancelSet();
    ResetAverage();
    LogUart_SendString("#SET done; PWM=0\r\n");
    g_forceStatus = 1U;
}

static void UpdateSet(TickType_t now)
{
    if (g_setStage == TUNING_CONSOLE_SET_STAGE_SETTLING) {
        if ((now - g_setStartTick) < pdMS_TO_TICKS(GMR_TUNING_SET_SETTLE_MS)) {
            return;
        }
        ResetAverage();
        g_setStage = TUNING_CONSOLE_SET_STAGE_SAMPLING;
        LogUart_SendString("#SET sampling\r\n");
    } else if ((g_setStage == TUNING_CONSOLE_SET_STAGE_SAMPLING) &&
        (g_averageSampleCount >= GMR_TUNING_SET_SAMPLE_WINDOWS)) {
        FinishSet();
    }
}

static void StartSet(int32_t left, int32_t right)
{
    if (g_setStage != TUNING_CONSOLE_SET_STAGE_IDLE) {
        LogUart_SendString("#ERR set busy; stop first\r\n");
        return;
    }
    if ((left < -GMR_TUNING_PWM_PERCENT_MAX) ||
        (left > GMR_TUNING_PWM_PERCENT_MAX) ||
        (right < -GMR_TUNING_PWM_PERCENT_MAX) ||
        (right > GMR_TUNING_PWM_PERCENT_MAX) ||
        ((left == 0) && (right == 0))) {
        LogUart_SendString("#ERR set range -100..100, one wheel nonzero\r\n");
        return;
    }
    GmrYawControl_Stop();
    g_setPwm[ENCODER_MOTOR_LEFT] = PercentToPwm(left);
    g_setPwm[ENCODER_MOTOR_RIGHT] = PercentToPwm(right);
    if (g_setPwm[ENCODER_MOTOR_LEFT] != 0) {
        g_setResult[ENCODER_MOTOR_LEFT] = TUNING_CONSOLE_SET_RESULT_RUNNING;
    }
    if (g_setPwm[ENCODER_MOTOR_RIGHT] != 0) {
        g_setResult[ENCODER_MOTOR_RIGHT] = TUNING_CONSOLE_SET_RESULT_RUNNING;
    }
    EncoderMotor_SetOpenLoopPwm(g_setPwm[ENCODER_MOTOR_LEFT],
        g_setPwm[ENCODER_MOTOR_RIGHT]);
    ResetAverage();
    g_setStage = TUNING_CONSOLE_SET_STAGE_SETTLING;
    g_setStartTick = xTaskGetTickCount();
    SendPair("set_pwm_pct", left, right);
    SendPair("set_pwm_raw", g_setPwm[ENCODER_MOTOR_LEFT],
        g_setPwm[ENCODER_MOTOR_RIGHT]);
    g_forceStatus = 1U;
}

static void SelectOled(const char *page)
{
    if (TextEquals(page, "ff") != 0U) {
        g_oledPage = TUNING_CONSOLE_OLED_FF;
    } else if (TextEquals(page, "start") != 0U) {
        g_oledPage = TUNING_CONSOLE_OLED_START;
    } else if (TextEquals(page, "speed") != 0U) {
        g_oledPage = TUNING_CONSOLE_OLED_SPEED;
    } else if (TextEquals(page, "pid") != 0U) {
        g_oledPage = TUNING_CONSOLE_OLED_PID;
    } else if (TextEquals(page, "yaw") != 0U) {
        g_oledPage = TUNING_CONSOLE_OLED_GIMBAL;
    } else {
        LogUart_SendString("#ERR oled expects ff|start|speed|pid|yaw\r\n");
        return;
    }
    g_forceStatus = 1U;
}

static void ExecuteLine(char *line)
{
    char *tokens[GMR_TUNING_TOKEN_MAX];
    uint8_t n;
    int32_t left;
    int32_t right;
    int32_t pwm1;
    int32_t count1;
    int32_t pwm2;
    int32_t count2;

    ToLower(line);
    n = Split(line, tokens, GMR_TUNING_TOKEN_MAX);
    if (n == 0U) {
        return;
    }
    if ((TextEquals(tokens[0], "help") != 0U) && (n == 1U)) {
        SendHelp();
    } else if ((TextEquals(tokens[0], "show") != 0U) && (n == 1U)) {
        SendDetails();
    } else if ((TextEquals(tokens[0], "stop") != 0U) && (n == 1U)) {
        CancelSet();
        GmrYawControl_Stop();
        EncoderMotor_SetOpenLoopPwm(0, 0);
        EncoderMotor_EnterCalibration();
        ResetAverage();
        g_forceStatus = 1U;
        LogUart_SendString("#OK stop\r\n");
    } else if ((TextEquals(tokens[0], "clear") != 0U) && (n == 1U)) {
        EncoderMotor_ClearIntegral();
        LogUart_SendString("#OK integral cleared\r\n");
    } else if ((TextEquals(tokens[0], "avg") != 0U) && (n == 1U)) {
        ResetAverage();
        LogUart_SendString("#OK average reset\r\n");
    } else if ((TextEquals(tokens[0], "oled") != 0U) && (n == 2U)) {
        SelectOled(tokens[1]);
    } else if ((TextEquals(tokens[0], "set") != 0U) && (n == 2U) &&
        (TextEquals(tokens[1], "clear") != 0U)) {
        CancelSet();
        GmrYawControl_Stop();
        ClearSetReferences();
        EncoderMotor_SetOpenLoopPwm(0, 0);
        LogUart_SendString("#OK set points cleared\r\n");
    } else if ((TextEquals(tokens[0], "set") != 0U) && (n == 3U)) {
        if (ParsePair(tokens, &left, &right) != 0U) {
            StartSet(left, right);
        }
    } else if ((TextEquals(tokens[0], "ffcalc") != 0U) && (n == 5U)) {
        if ((ParseInt32(tokens[1], &pwm1) != 0U) &&
            (ParseInt32(tokens[2], &count1) != 0U) &&
            (ParseInt32(tokens[3], &pwm2) != 0U) &&
            (ParseInt32(tokens[4], &count2) != 0U)) {
            CalculateFf(pwm1, count1, pwm2, count2);
        } else {
            LogUart_SendString("#ERR ffcalc expects integers: P1 C1 P2 C2\r\n");
        }
    } else if ((TextEquals(tokens[0], "pwm") != 0U) && (n == 3U)) {
        if (ParsePair(tokens, &left, &right) == 0U) {
            return;
        }
        if ((left < -100) || (left > 100) || (right < -100) ||
            (right > 100)) {
            LogUart_SendString("#ERR pwm range -100..100 percent\r\n");
            return;
        }
        CancelSet();
        GmrYawControl_Stop();
        EncoderMotor_SetOpenLoopPwm(PercentToPwm(left), PercentToPwm(right));
        ResetAverage();
        SendPair("pwm_pct", left, right);
    } else if ((TextEquals(tokens[0], "target") != 0U) && (n == 3U)) {
        if (ParsePair(tokens, &left, &right) == 0U) {
            return;
        }
        if ((left < -(int32_t)CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) ||
            (left > (int32_t)CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) ||
            (right < -(int32_t)CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) ||
            (right > (int32_t)CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD)) {
            LogUart_SendString("#ERR target range is -100..100 count/20ms\r\n");
            return;
        }
        CancelSet();
        GmrYawControl_Stop();
        EncoderMotor_SetCalibrationTargets((int16_t)left, (int16_t)right);
        ResetAverage();
        SendPair("target_count", left, right);
    } else if ((TextEquals(tokens[0], "angle") != 0U) && (n == 2U)) {
        if (ParseInt32(tokens[1], &left) == 0U) {
            LogUart_SendString("#ERR angle expects integer degrees\r\n");
            return;
        }
        if ((left < -(int32_t)GMR_M0_YAW_COMMAND_MAX_DEGREES) ||
            (left > (int32_t)GMR_M0_YAW_COMMAND_MAX_DEGREES)) {
            LogUart_SendString("#ERR angle range -180..180 degrees\r\n");
            return;
        }
        CancelSet();
        if (GmrYawControl_StartRelative((int16_t)left) == 0U) {
            LogUart_SendString("#ERR M0 yaw unavailable or stale\r\n");
            return;
        }
        g_oledPage = TUNING_CONSOLE_OLED_GIMBAL;
        ResetAverage();
        LogUart_SendString("#OK relative_angle_deg=");
        LogUart_SendSigned(left);
        LogUart_SendString("\r\n");
        g_forceStatus = 1U;
    } else if (((TextEquals(tokens[0], "ff") != 0U) ||
        (TextEquals(tokens[0], "kp") != 0U) ||
        (TextEquals(tokens[0], "ki") != 0U)) && (n == 3U)) {
        if (ParsePair(tokens, &left, &right) != 0U) {
            ApplyGain(tokens[0], left, right);
        }
    } else if (((TextEquals(tokens[0], "start") != 0U) ||
        (TextEquals(tokens[0], "runstart") != 0U) ||
        (TextEquals(tokens[0], "ilim") != 0U)) && (n == 3U)) {
        if (ParsePair(tokens, &left, &right) != 0U) {
            ApplyPwmParameter(tokens[0], left, right);
        }
    } else {
        LogUart_SendString("#ERR bad command, type help\r\n");
    }
}

static void ProcessRx(void)
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
                ExecuteLine(g_line);
                g_lineLength = 0U;
            }
        } else if ((data == 8U) || (data == 127U)) {
            if ((g_discardLine == 0U) && (g_lineLength > 0U)) {
                --g_lineLength;
            }
        } else if (g_discardLine == 0U) {
            if (g_lineLength < (GMR_TUNING_LINE_MAX - 1U)) {
                g_line[g_lineLength++] = (char)data;
            } else {
                g_discardLine = 1U;
            }
        }
    }
}

static void TuningConsole_StartWithPage(TuningConsoleOledPage page)
{
    if (g_active != 0U) {
        return;
    }
    g_active = 1U;
    g_lineLength = 0U;
    g_discardLine = 0U;
    g_forceStatus = 0U;
    g_oledPage = page;
    CancelSet();
    ClearSetReferences();
    LogUart_ClearRx();
    GmrYawControl_Stop();
    EncoderMotor_EnterCalibration();
    if (page == TUNING_CONSOLE_OLED_GIMBAL) {
        GmrYawControl_StartHold();
    }
    ResetAverage();
    g_lastStatusTick = xTaskGetTickCount();
    LogUart_SendString((page == TUNING_CONSOLE_OLED_GIMBAL) ?
        "# GMR Task5 M0 heading hold armed; waiting for fresh yaw\r\n" :
        "# GMR Task2 PID calibration ready\r\n");
    SendHelp();
    SendStatus();
}

void TuningConsole_Start(void)
{
    TuningConsole_StartWithPage(TUNING_CONSOLE_OLED_FF);
}

void TuningConsole_StartYaw(void)
{
    TuningConsole_StartWithPage(TUNING_CONSOLE_OLED_GIMBAL);
}

void TuningConsole_Stop(void)
{
    if (g_active == 0U) {
        return;
    }
    GmrYawControl_Stop();
    CancelSet();
    EncoderMotor_ExitCalibration();
    ClearSetReferences();
    g_active = 0U;
    g_lineLength = 0U;
    g_discardLine = 0U;
    g_forceStatus = 0U;
    LogUart_SendString("# GMR tuning stopped; PWM=0\r\n");
}

void TuningConsole_Task(void)
{
    TickType_t now;

    if (g_active == 0U) {
        return;
    }
    ProcessRx();
    UpdateAverage();
    now = xTaskGetTickCount();
    UpdateSet(now);
    if ((g_forceStatus != 0U) ||
        ((now - g_lastStatusTick) >=
            pdMS_TO_TICKS(GMR_TUNING_STATUS_PERIOD_MS))) {
        g_forceStatus = 0U;
        g_lastStatusTick = now;
        SendStatus();
    }
}

void TuningConsole_ChassisControlPeriod(void)
{
    GmrYawControl_RunControlPeriod();
}

uint8_t TuningConsole_IsActive(void)
{
    return g_active;
}

void TuningConsole_GetDisplayStatus(TuningConsoleDisplayStatus *status)
{
    EncoderMotorSnapshot s;
    GmrYawControlSnapshot yaw;
#if CAR_LIBRARY_H7_IMU_ENABLED
    H7GyroLinkFeedback h7;
    TickType_t now;
#endif

    if (status == 0) {
        return;
    }
    EncoderMotor_GetSnapshot(&s);
    GmrYawControl_GetSnapshot(&yaw);
    status->oledPage = g_oledPage;
    status->setStage = g_setStage;
    status->leftResult = g_setResult[ENCODER_MOTOR_LEFT];
    status->rightResult = g_setResult[ENCODER_MOTOR_RIGHT];
    status->sampleCount = g_averageSampleCount;
    status->sampleTarget = GMR_TUNING_SET_SAMPLE_WINDOWS;
    status->leftFfQ1024 = s.tuning[ENCODER_MOTOR_LEFT].ffQ1024;
    status->rightFfQ1024 = s.tuning[ENCODER_MOTOR_RIGHT].ffQ1024;
    status->leftStartPercent =
        PwmToPercent(s.tuning[ENCODER_MOTOR_LEFT].startPwm);
    status->rightStartPercent =
        PwmToPercent(s.tuning[ENCODER_MOTOR_RIGHT].startPwm);
    status->leftRunStartPercent =
        PwmToPercent(s.tuning[ENCODER_MOTOR_LEFT].runStartPwm);
    status->rightRunStartPercent =
        PwmToPercent(s.tuning[ENCODER_MOTOR_RIGHT].runStartPwm);
    status->leftPwmPercent = PwmToPercent(s.outputPwm[ENCODER_MOTOR_LEFT]);
    status->rightPwmPercent = PwmToPercent(s.outputPwm[ENCODER_MOTOR_RIGHT]);
    status->leftTargetCounts = s.targetCounts[ENCODER_MOTOR_LEFT];
    status->rightTargetCounts = s.targetCounts[ENCODER_MOTOR_RIGHT];
    status->leftFeedbackCounts = s.feedbackCounts[ENCODER_MOTOR_LEFT];
    status->rightFeedbackCounts = s.feedbackCounts[ENCODER_MOTOR_RIGHT];
    status->leftAverageCounts = GetAverage(ENCODER_MOTOR_LEFT);
    status->rightAverageCounts = GetAverage(ENCODER_MOTOR_RIGHT);
    status->grayMask = 0U;
    status->gimbalMode = yaw.active;
    status->visionMode = 0U;
    status->visionHasFrame = 0U;
    status->visionFrameCount = 0U;
    status->visionRawX = 0;
    status->visionRawY = 0;
    status->visionStageScaleX10 = 0;
    status->visionCommandX = 0;
    status->visionCommandY = 0;
    status->visionYawGainQ1024 = 0U;
    status->gimbalState = yaw.active;
    status->gimbalHoldEnabled = yaw.targetReached;
    status->gimbalFeedForwardEnabled = 0U;
    status->gimbalFeedbackFresh = yaw.sensorFresh;
    status->gimbalFeedForwardFresh = 0U;
    status->gimbalCalibrationCount = 0U;
    status->gimbalCalibrationTarget = 0U;
    status->gimbalYawX100 = yaw.yawX100;
    status->gimbalRateX100PerSec = yaw.yawRateX100PerSec;
    status->gimbalAngleErrorX100 = yaw.errorX100;
    status->gimbalCommandSps = yaw.wheelCommandCounts;
#if CAR_LIBRARY_H7_IMU_ENABLED
    now = xTaskGetTickCount();
    (void)H7GyroLink_GetFeedback(&h7);
    status->h7ImuAgeMs = (h7.angleFrameCount == 0U) ? 0xFFFFFFFFUL :
        (uint32_t)(now - h7.angleFrameTick);
    status->h7ImuFresh = (uint8_t)((h7.angleFrameCount != 0U) &&
        (status->h7ImuAgeMs <= (uint32_t)GMR_H7_IMU_STALE_MS));
    status->h7RollX100 = h7.rollX100;
    status->h7PitchX100 = h7.pitchX100;
    status->h7YawX100 = h7.yawUnwrappedX100;
    status->h7YawRateX100PerSec = h7.yawRateX100PerSec;
#else
    status->h7ImuFresh = 0U;
    status->h7ImuAgeMs = 0xFFFFFFFFUL;
    status->h7RollX100 = 0;
    status->h7PitchX100 = 0;
    status->h7YawX100 = 0;
    status->h7YawRateX100PerSec = 0;
#endif
}

#endif
