#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "gmr_bluetooth_mission.h"

#include "bluetooth_config.h"
#include "bluetooth_service.h"
#include "control_config.h"
#include "encoder_motor.h"
#include "motor_enable.h"
#include "motor_no_yaw.h"

#define GMR_BT_SIGNAL_READY             (0x0601U)
#define GMR_BT_SIGNAL_START             (0x0602U)
#define GMR_BT_SIGNAL_START_ACK         (0x0603U)
#define GMR_BT_SIGNAL_ENCODER_DELTA     (0x0604U)
#define GMR_BT_SIGNAL_OVER              (0x0605U)
#define GMR_BT_SIGNAL_OVER_ACK          (0x0606U)
#define GMR_BT_SIGNAL_DONE              (0x0607U)
#define GMR_BT_PROTOCOL_VERSION         (2U)

#if ((GMR_BLUETOOTH_MISSION_SAMPLE_PERIOD_MS == 0U) || \
    ((GMR_BLUETOOTH_MISSION_SAMPLE_PERIOD_MS % \
        CHASSIS_CONTROL_PERIOD_MS) != 0U))
#error "Task6 Bluetooth sample period must be a multiple of chassis period"
#endif

#if ((GMR_BLUETOOTH_MISSION_MAX_SAMPLES < 2U) || \
    (GMR_BLUETOOTH_MISSION_MAX_SAMPLES > 65535U))
#error "Task6 Bluetooth sample capacity must fit uint16_t"
#endif

typedef struct {
    int16_t left;
    int16_t right;
} GmrBluetoothEncoderDelta;

static volatile uint8_t g_active;
static volatile GmrBluetoothMissionPhase g_phase;
static volatile GmrBluetoothMissionError g_error;
static volatile GmrBluetoothMissionResult g_result;
static volatile uint16_t g_sampleCount;
static volatile uint16_t g_replayIndex;
static uint8_t g_linkEstablished;
static uint8_t g_masterTrackingStarted;
static uint16_t g_sampleElapsedMs;
static uint32_t g_retryElapsedMs;
static uint32_t g_sessionSeed;
static int32_t g_sessionToken;

#if (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_MASTER)
static int32_t g_masterLastLeftCount;
static int32_t g_masterLastRightCount;
#endif

#if (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_SLAVE)
static GmrBluetoothEncoderDelta
    g_samples[GMR_BLUETOOTH_MISSION_MAX_SAMPLES];
static int32_t g_replayTargetLeftCount;
static int32_t g_replayTargetRightCount;
static volatile int32_t g_replayLeftErrorCounts;
static volatile int32_t g_replayRightErrorCounts;
static uint32_t g_replaySegmentElapsedMs;
static uint8_t g_replayTargetLoaded;
#endif

static uint32_t GmrBluetoothMission_SaturatingAdd(uint32_t value,
    uint32_t increment)
{
    return (value > (0xFFFFFFFFU - increment)) ?
        0xFFFFFFFFU : value + increment;
}

static uint8_t GmrBluetoothMission_SessionFlag(void)
{
    return (uint8_t)((uint32_t)g_sessionToken & 0xFFU);
}

static void GmrBluetoothMission_StopChassis(void)
{
    MotorNoYaw_Stop();
    EncoderMotor_SetPeriodTargets(0, 0);
    EncoderMotor_Stop();
    MotorEnable_SetChassis(0U);
}

static void GmrBluetoothMission_SetError(GmrBluetoothMissionError error)
{
    if (g_error == GMR_BLUETOOTH_MISSION_ERROR_NONE) {
        g_error = error;
    }
    g_phase = GMR_BLUETOOTH_MISSION_PHASE_ERROR;
    g_result = GMR_BLUETOOTH_MISSION_RESULT_ERROR;
    GmrBluetoothMission_StopChassis();
}

static uint8_t GmrBluetoothMission_Send(uint16_t id, int32_t value,
    uint8_t flags)
{
    if (BluetoothService_SendSignal(id, value, flags) == 0U) {
        GmrBluetoothMission_SetError(
            GMR_BLUETOOTH_MISSION_ERROR_TX_QUEUE);
        return 0U;
    }
    return 1U;
}

static int32_t GmrBluetoothMission_PackDelta(int16_t left, int16_t right)
{
    uint32_t packed = (uint32_t)(uint16_t)left;

    packed |= (uint32_t)((uint32_t)(uint16_t)right << 16U);
    return (int32_t)packed;
}

#if (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_SLAVE)
static GmrBluetoothEncoderDelta GmrBluetoothMission_UnpackDelta(
    int32_t value)
{
    uint32_t packed = (uint32_t)value;
    GmrBluetoothEncoderDelta sample;

    sample.left = (int16_t)(uint16_t)(packed & 0xFFFFU);
    sample.right = (int16_t)(uint16_t)(packed >> 16U);
    return sample;
}
#endif

static void GmrBluetoothMission_DrainSignals(void)
{
    BluetoothSignal signal;

    while (BluetoothService_TakeSignal(&signal) != 0U) {
    }
}

#if (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_MASTER)
static uint8_t GmrBluetoothMission_MasterSendStart(void)
{
    if (GmrBluetoothMission_Send(GMR_BT_SIGNAL_START, g_sessionToken,
        GMR_BT_PROTOCOL_VERSION) == 0U) {
        return 0U;
    }
    g_phase = GMR_BLUETOOTH_MISSION_PHASE_WAIT_START_ACK;
    g_retryElapsedMs = 0U;
    return 1U;
}

static uint8_t GmrBluetoothMission_MasterSendDelta(int16_t left,
    int16_t right)
{
    uint16_t sampleIndex = g_sampleCount;

    if (sampleIndex >= GMR_BLUETOOTH_MISSION_MAX_SAMPLES) {
        GmrBluetoothMission_SetError(
            GMR_BLUETOOTH_MISSION_ERROR_BUFFER_FULL);
        return 0U;
    }
    if (GmrBluetoothMission_Send(GMR_BT_SIGNAL_ENCODER_DELTA,
        GmrBluetoothMission_PackDelta(left, right),
        (uint8_t)sampleIndex) == 0U) {
        return 0U;
    }
    g_sampleCount = (uint16_t)(sampleIndex + 1U);
    return 1U;
}

/* 记录最近50ms实际走过的左右编码数；静止窗口不占用回放缓存。 */
static uint8_t GmrBluetoothMission_MasterCaptureDelta(void)
{
    int32_t leftCount;
    int32_t rightCount;
    int64_t leftDelta;
    int64_t rightDelta;

    EncoderMotor_GetTotalCounts(&leftCount, &rightCount);
    leftDelta = (int64_t)leftCount - (int64_t)g_masterLastLeftCount;
    rightDelta = (int64_t)rightCount - (int64_t)g_masterLastRightCount;
    g_masterLastLeftCount = leftCount;
    g_masterLastRightCount = rightCount;
    if ((leftDelta < (int64_t)INT16_MIN) ||
        (leftDelta > (int64_t)INT16_MAX) ||
        (rightDelta < (int64_t)INT16_MIN) ||
        (rightDelta > (int64_t)INT16_MAX)) {
        GmrBluetoothMission_SetError(
            GMR_BLUETOOTH_MISSION_ERROR_DISTANCE_RANGE);
        return 0U;
    }
    if ((leftDelta == 0) && (rightDelta == 0)) {
        return 1U;
    }
    return GmrBluetoothMission_MasterSendDelta((int16_t)leftDelta,
        (int16_t)rightDelta);
}

static uint8_t GmrBluetoothMission_MasterSendOver(void)
{
    if (GmrBluetoothMission_Send(GMR_BT_SIGNAL_OVER,
        (int32_t)g_sampleCount,
        GmrBluetoothMission_SessionFlag()) == 0U) {
        return 0U;
    }
    g_phase = GMR_BLUETOOTH_MISSION_PHASE_WAIT_OVER_ACK;
    g_retryElapsedMs = 0U;
    return 1U;
}

static void GmrBluetoothMission_MasterHandleSignal(
    const BluetoothSignal *signal)
{
    if ((g_phase == GMR_BLUETOOTH_MISSION_PHASE_WAIT_READY) &&
        (signal->id == GMR_BT_SIGNAL_READY)) {
        if ((signal->flags != GMR_BT_PROTOCOL_VERSION) ||
            (signal->value !=
                (int32_t)GMR_BLUETOOTH_MISSION_MAX_SAMPLES)) {
            GmrBluetoothMission_SetError(
                GMR_BLUETOOTH_MISSION_ERROR_PROTOCOL);
            return;
        }
        (void)GmrBluetoothMission_MasterSendStart();
    } else if ((g_phase ==
        GMR_BLUETOOTH_MISSION_PHASE_WAIT_START_ACK) &&
        (signal->id == GMR_BT_SIGNAL_START_ACK) &&
        (signal->value == g_sessionToken) &&
        (signal->flags == GMR_BT_PROTOCOL_VERSION)) {
        g_phase = GMR_BLUETOOTH_MISSION_PHASE_RECORDING;
        g_retryElapsedMs = 0U;
    } else if ((g_phase ==
        GMR_BLUETOOTH_MISSION_PHASE_WAIT_OVER_ACK) &&
        (signal->id == GMR_BT_SIGNAL_OVER_ACK) &&
        (signal->value == (int32_t)g_sampleCount) &&
        (signal->flags == GmrBluetoothMission_SessionFlag())) {
        g_phase = GMR_BLUETOOTH_MISSION_PHASE_DONE;
        g_result = GMR_BLUETOOTH_MISSION_RESULT_FINISHED;
    }
}

static void GmrBluetoothMission_MasterCommTask(uint32_t elapsedMs)
{
    BluetoothSignal signal;

    while ((g_result == GMR_BLUETOOTH_MISSION_RESULT_RUNNING) &&
        (BluetoothService_TakeSignal(&signal) != 0U)) {
        GmrBluetoothMission_MasterHandleSignal(&signal);
    }

    if ((g_phase != GMR_BLUETOOTH_MISSION_PHASE_WAIT_START_ACK) &&
        (g_phase != GMR_BLUETOOTH_MISSION_PHASE_WAIT_OVER_ACK)) {
        return;
    }
    g_retryElapsedMs = GmrBluetoothMission_SaturatingAdd(
        g_retryElapsedMs, elapsedMs);
    if (g_retryElapsedMs < GMR_BLUETOOTH_MISSION_RETRY_PERIOD_MS) {
        return;
    }
    g_retryElapsedMs = 0U;
    if (g_phase == GMR_BLUETOOTH_MISSION_PHASE_WAIT_START_ACK) {
        (void)GmrBluetoothMission_MasterSendStart();
    } else {
        (void)GmrBluetoothMission_MasterSendOver();
    }
}

static void GmrBluetoothMission_MasterControlPeriod(void)
{
    if (g_phase != GMR_BLUETOOTH_MISSION_PHASE_RECORDING) {
        return;
    }
    if (g_masterTrackingStarted == 0U) {
        MotorNoYaw_Start();
        EncoderMotor_GetTotalCounts(&g_masterLastLeftCount,
            &g_masterLastRightCount);
        g_masterTrackingStarted = 1U;
        g_sampleElapsedMs = 0U;
    }

    MotorNoYaw_Task();
    if ((MotorNoYaw_IsRunning() == 0U) ||
        (MotorNoYaw_GetState() == MOTOR_NO_YAW_STATE_STOP)) {
        GmrBluetoothMission_SetError(
            GMR_BLUETOOTH_MISSION_ERROR_LINE_FOLLOW);
        return;
    }
    if (MotorNoYaw_GetTurnCount() >=
        GMR_BLUETOOTH_MISSION_TARGET_TURNS) {
        GmrBluetoothMission_StopChassis();
        if ((GmrBluetoothMission_MasterCaptureDelta() != 0U) &&
            (g_sampleCount == 0U)) {
            GmrBluetoothMission_SetError(
                GMR_BLUETOOTH_MISSION_ERROR_PROTOCOL);
        } else if ((g_result == GMR_BLUETOOTH_MISSION_RESULT_RUNNING) &&
            (GmrBluetoothMission_MasterSendOver() != 0U)) {
            g_masterTrackingStarted = 0U;
        }
        return;
    }

    g_sampleElapsedMs = (uint16_t)(g_sampleElapsedMs +
        CHASSIS_CONTROL_PERIOD_MS);
    if (g_sampleElapsedMs < GMR_BLUETOOTH_MISSION_SAMPLE_PERIOD_MS) {
        return;
    }
    g_sampleElapsedMs = 0U;
    if (g_sampleCount >=
        (GMR_BLUETOOTH_MISSION_MAX_SAMPLES - 1U)) {
        GmrBluetoothMission_SetError(
            GMR_BLUETOOTH_MISSION_ERROR_BUFFER_FULL);
        return;
    }
    (void)GmrBluetoothMission_MasterCaptureDelta();
}
#endif

#if (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_SLAVE)
static void GmrBluetoothMission_SlaveAcceptStart(
    const BluetoothSignal *signal)
{
    g_sessionToken = signal->value;
    g_sampleCount = 0U;
    g_replayIndex = 0U;
    g_replayTargetLeftCount = 0;
    g_replayTargetRightCount = 0;
    g_replayLeftErrorCounts = 0;
    g_replayRightErrorCounts = 0;
    g_replaySegmentElapsedMs = 0U;
    g_replayTargetLoaded = 0U;
    if (GmrBluetoothMission_Send(GMR_BT_SIGNAL_START_ACK,
        g_sessionToken, GMR_BT_PROTOCOL_VERSION) != 0U) {
        g_phase = GMR_BLUETOOTH_MISSION_PHASE_RECEIVING;
    }
}

static void GmrBluetoothMission_SlaveHandleSample(
    const BluetoothSignal *signal)
{
    uint16_t sampleIndex = g_sampleCount;

    if (signal->flags != (uint8_t)sampleIndex) {
        GmrBluetoothMission_SetError(
            GMR_BLUETOOTH_MISSION_ERROR_SAMPLE_ORDER);
        return;
    }
    if (sampleIndex >= GMR_BLUETOOTH_MISSION_MAX_SAMPLES) {
        GmrBluetoothMission_SetError(
            GMR_BLUETOOTH_MISSION_ERROR_BUFFER_FULL);
        return;
    }
    g_samples[sampleIndex] =
        GmrBluetoothMission_UnpackDelta(signal->value);
    g_sampleCount = (uint16_t)(sampleIndex + 1U);
}

static void GmrBluetoothMission_SlaveHandleOver(
    const BluetoothSignal *signal)
{
    uint16_t sampleCount = g_sampleCount;

    if ((signal->flags != GmrBluetoothMission_SessionFlag()) ||
        (signal->value != (int32_t)sampleCount) ||
        (sampleCount == 0U)) {
        GmrBluetoothMission_SetError(
            GMR_BLUETOOTH_MISSION_ERROR_PROTOCOL);
        return;
    }
    GmrBluetoothMission_StopChassis();
    EncoderMotor_ResetAllCounts();
    g_replayTargetLeftCount = 0;
    g_replayTargetRightCount = 0;
    g_replayLeftErrorCounts = 0;
    g_replayRightErrorCounts = 0;
    if (GmrBluetoothMission_Send(GMR_BT_SIGNAL_OVER_ACK,
        (int32_t)sampleCount,
        GmrBluetoothMission_SessionFlag()) != 0U) {
        g_replayIndex = 0U;
        g_replaySegmentElapsedMs = 0U;
        g_replayTargetLoaded = 0U;
        g_phase = GMR_BLUETOOTH_MISSION_PHASE_REPLAY;
    }
}

static void GmrBluetoothMission_SlaveHandleSignal(
    const BluetoothSignal *signal)
{
    if (signal->id == GMR_BT_SIGNAL_START) {
        if (signal->flags != GMR_BT_PROTOCOL_VERSION) {
            GmrBluetoothMission_SetError(
                GMR_BLUETOOTH_MISSION_ERROR_PROTOCOL);
        } else if ((g_phase ==
            GMR_BLUETOOTH_MISSION_PHASE_RECEIVING) &&
            (signal->value == g_sessionToken)) {
            (void)GmrBluetoothMission_Send(GMR_BT_SIGNAL_START_ACK,
                g_sessionToken, GMR_BT_PROTOCOL_VERSION);
        } else if ((g_phase == GMR_BLUETOOTH_MISSION_PHASE_WAIT_START) ||
            (g_phase == GMR_BLUETOOTH_MISSION_PHASE_RECEIVING)) {
            GmrBluetoothMission_SlaveAcceptStart(signal);
        }
        return;
    }
    if ((g_phase == GMR_BLUETOOTH_MISSION_PHASE_RECEIVING) &&
        (signal->id == GMR_BT_SIGNAL_ENCODER_DELTA)) {
        GmrBluetoothMission_SlaveHandleSample(signal);
    } else if ((g_phase ==
        GMR_BLUETOOTH_MISSION_PHASE_RECEIVING) &&
        (signal->id == GMR_BT_SIGNAL_OVER)) {
        GmrBluetoothMission_SlaveHandleOver(signal);
    } else if ((g_phase == GMR_BLUETOOTH_MISSION_PHASE_REPLAY) &&
        (signal->id == GMR_BT_SIGNAL_OVER) &&
        (signal->value == (int32_t)g_sampleCount) &&
        (signal->flags == GmrBluetoothMission_SessionFlag())) {
        (void)GmrBluetoothMission_Send(GMR_BT_SIGNAL_OVER_ACK,
            (int32_t)g_sampleCount,
            GmrBluetoothMission_SessionFlag());
    }
}

static void GmrBluetoothMission_SlaveCommTask(uint32_t elapsedMs)
{
    BluetoothSignal signal;

    while ((g_result == GMR_BLUETOOTH_MISSION_RESULT_RUNNING) &&
        (BluetoothService_TakeSignal(&signal) != 0U)) {
        GmrBluetoothMission_SlaveHandleSignal(&signal);
    }

    if (g_phase != GMR_BLUETOOTH_MISSION_PHASE_WAIT_START) {
        return;
    }
    g_retryElapsedMs = GmrBluetoothMission_SaturatingAdd(
        g_retryElapsedMs, elapsedMs);
    if (g_retryElapsedMs >= GMR_BLUETOOTH_MISSION_READY_PERIOD_MS) {
        g_retryElapsedMs = 0U;
        (void)GmrBluetoothMission_Send(GMR_BT_SIGNAL_READY,
            (int32_t)GMR_BLUETOOTH_MISSION_MAX_SAMPLES,
            GMR_BT_PROTOCOL_VERSION);
    }
}

static int32_t GmrBluetoothMission_ClampDisplayError(int64_t error)
{
    if (error > (int64_t)INT32_MAX) {
        return INT32_MAX;
    }
    if (error < (int64_t)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)error;
}

static uint8_t GmrBluetoothMission_DistanceReached(int64_t leftError,
    int64_t rightError)
{
    int64_t tolerance =
        (int64_t)GMR_BLUETOOTH_DISTANCE_TOLERANCE_COUNTS;

    return (uint8_t)(((leftError >= -tolerance) &&
        (leftError <= tolerance) &&
        (rightError >= -tolerance) &&
        (rightError <= tolerance)) ? 1U : 0U);
}

static int16_t GmrBluetoothMission_PositionErrorToSpeed(int64_t error)
{
    int64_t magnitude = (error < 0) ? -error : error;
    int64_t speed;

    if (magnitude <=
        (int64_t)GMR_BLUETOOTH_DISTANCE_TOLERANCE_COUNTS) {
        return 0;
    }
    speed = (magnitude *
        (int64_t)GMR_BLUETOOTH_DISTANCE_KP_Q1024) /
        (int64_t)CHASSIS_Q1024_SCALE;
    if (speed <
        (int64_t)GMR_BLUETOOTH_DISTANCE_MIN_SPEED_COUNTS_PER_PERIOD) {
        speed =
            (int64_t)GMR_BLUETOOTH_DISTANCE_MIN_SPEED_COUNTS_PER_PERIOD;
    }
    if (speed >
        (int64_t)GMR_BLUETOOTH_DISTANCE_MAX_SPEED_COUNTS_PER_PERIOD) {
        speed =
            (int64_t)GMR_BLUETOOTH_DISTANCE_MAX_SPEED_COUNTS_PER_PERIOD;
    }
    return (int16_t)((error < 0) ? -speed : speed);
}

static uint8_t GmrBluetoothMission_SlaveLoadTarget(uint16_t sampleIndex)
{
    int64_t nextLeft = (int64_t)g_replayTargetLeftCount +
        (int64_t)g_samples[sampleIndex].left;
    int64_t nextRight = (int64_t)g_replayTargetRightCount +
        (int64_t)g_samples[sampleIndex].right;

    if ((nextLeft < (int64_t)INT32_MIN) ||
        (nextLeft > (int64_t)INT32_MAX) ||
        (nextRight < (int64_t)INT32_MIN) ||
        (nextRight > (int64_t)INT32_MAX)) {
        GmrBluetoothMission_SetError(
            GMR_BLUETOOTH_MISSION_ERROR_DISTANCE_RANGE);
        return 0U;
    }
    g_replayTargetLeftCount = (int32_t)nextLeft;
    g_replayTargetRightCount = (int32_t)nextRight;
    g_replaySegmentElapsedMs = 0U;
    g_replayTargetLoaded = 1U;
    return 1U;
}

static void GmrBluetoothMission_SlaveFinishReplay(void)
{
    EncoderMotor_SetPeriodTargets(0, 0);
    MotorEnable_SetChassis(0U);
    g_replayLeftErrorCounts = 0;
    g_replayRightErrorCounts = 0;
    if (GmrBluetoothMission_Send(GMR_BT_SIGNAL_DONE,
        (int32_t)g_sampleCount,
        GmrBluetoothMission_SessionFlag()) != 0U) {
        g_phase = GMR_BLUETOOTH_MISSION_PHASE_DONE;
        g_result = GMR_BLUETOOTH_MISSION_RESULT_FINISHED;
    }
}

static void GmrBluetoothMission_SlaveControlPeriod(void)
{
    int32_t leftCount;
    int32_t rightCount;
    int64_t leftError;
    int64_t rightError;

    if (g_phase != GMR_BLUETOOTH_MISSION_PHASE_REPLAY) {
        return;
    }

    EncoderMotor_GetTotalCounts(&leftCount, &rightCount);
    for (;;) {
        leftError = (int64_t)g_replayTargetLeftCount -
            (int64_t)leftCount;
        rightError = (int64_t)g_replayTargetRightCount -
            (int64_t)rightCount;
        g_replayLeftErrorCounts =
            GmrBluetoothMission_ClampDisplayError(leftError);
        g_replayRightErrorCounts =
            GmrBluetoothMission_ClampDisplayError(rightError);
        if ((g_replayTargetLoaded != 0U) &&
            (GmrBluetoothMission_DistanceReached(leftError,
                rightError) == 0U)) {
            break;
        }
        if (g_replayTargetLoaded != 0U) {
            g_replayIndex = (uint16_t)(g_replayIndex + 1U);
            g_replayTargetLoaded = 0U;
        }
        if (g_replayIndex >= g_sampleCount) {
            GmrBluetoothMission_SlaveFinishReplay();
            return;
        }
        if (GmrBluetoothMission_SlaveLoadTarget(g_replayIndex) == 0U) {
            return;
        }
    }

    g_replaySegmentElapsedMs = GmrBluetoothMission_SaturatingAdd(
        g_replaySegmentElapsedMs, CHASSIS_CONTROL_PERIOD_MS);
    if (g_replaySegmentElapsedMs >=
        GMR_BLUETOOTH_DISTANCE_SEGMENT_TIMEOUT_MS) {
        GmrBluetoothMission_SetError(
            GMR_BLUETOOTH_MISSION_ERROR_REPLAY_TIMEOUT);
        return;
    }
    EncoderMotor_SetPeriodTargets(
        GmrBluetoothMission_PositionErrorToSpeed(leftError),
        GmrBluetoothMission_PositionErrorToSpeed(rightError));
    MotorEnable_SetChassis(1U);
}
#endif

void GmrBluetoothMission_Init(void)
{
    g_active = 0U;
    g_phase = GMR_BLUETOOTH_MISSION_PHASE_IDLE;
    g_error = GMR_BLUETOOTH_MISSION_ERROR_NONE;
    g_result = GMR_BLUETOOTH_MISSION_RESULT_RUNNING;
    g_sampleCount = 0U;
    g_replayIndex = 0U;
    g_linkEstablished = 0U;
    g_masterTrackingStarted = 0U;
    g_sampleElapsedMs = 0U;
    g_retryElapsedMs = 0U;
    g_sessionSeed = 0U;
    g_sessionToken = 0;
#if (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_MASTER)
    g_masterLastLeftCount = 0;
    g_masterLastRightCount = 0;
#elif (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_SLAVE)
    g_replayTargetLeftCount = 0;
    g_replayTargetRightCount = 0;
    g_replayLeftErrorCounts = 0;
    g_replayRightErrorCounts = 0;
    g_replaySegmentElapsedMs = 0U;
    g_replayTargetLoaded = 0U;
#endif
}

void GmrBluetoothMission_Start(void)
{
    GmrBluetoothMission_DrainSignals();
    GmrBluetoothMission_StopChassis();
    g_active = 1U;
    g_phase = GMR_BLUETOOTH_MISSION_PHASE_WAIT_LINK;
    g_error = GMR_BLUETOOTH_MISSION_ERROR_NONE;
    g_result = GMR_BLUETOOTH_MISSION_RESULT_RUNNING;
    g_sampleCount = 0U;
    g_replayIndex = 0U;
    g_linkEstablished = 0U;
    g_masterTrackingStarted = 0U;
    g_sampleElapsedMs = 0U;
    g_retryElapsedMs = 0U;
#if (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_MASTER)
    g_masterLastLeftCount = 0;
    g_masterLastRightCount = 0;
    ++g_sessionSeed;
    g_sessionToken = (int32_t)(0x06000000U |
        (g_sessionSeed & 0x00FFFFFFU));
#elif (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_SLAVE)
    g_replayTargetLeftCount = 0;
    g_replayTargetRightCount = 0;
    g_replayLeftErrorCounts = 0;
    g_replayRightErrorCounts = 0;
    g_replaySegmentElapsedMs = 0U;
    g_replayTargetLoaded = 0U;
    g_sessionToken = 0;
#else
    GmrBluetoothMission_SetError(
        GMR_BLUETOOTH_MISSION_ERROR_DISABLED);
#endif
}

void GmrBluetoothMission_Stop(void)
{
    g_active = 0U;
    GmrBluetoothMission_StopChassis();
    if ((g_phase != GMR_BLUETOOTH_MISSION_PHASE_DONE) &&
        (g_phase != GMR_BLUETOOTH_MISSION_PHASE_ERROR)) {
        g_phase = GMR_BLUETOOTH_MISSION_PHASE_IDLE;
    }
}

void GmrBluetoothMission_CommTask(uint32_t elapsedMs)
{
    if ((g_active == 0U) ||
        (g_result != GMR_BLUETOOTH_MISSION_RESULT_RUNNING)) {
        return;
    }
    if (BluetoothService_IsConnected() == 0U) {
        if (g_linkEstablished != 0U) {
            GmrBluetoothMission_SetError(
                GMR_BLUETOOTH_MISSION_ERROR_LINK_LOST);
        }
        return;
    }
    if (g_linkEstablished == 0U) {
        g_linkEstablished = 1U;
        g_retryElapsedMs = GMR_BLUETOOTH_MISSION_READY_PERIOD_MS;
#if (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_MASTER)
        g_phase = GMR_BLUETOOTH_MISSION_PHASE_WAIT_READY;
#elif (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_SLAVE)
        g_phase = GMR_BLUETOOTH_MISSION_PHASE_WAIT_START;
#endif
    }

#if (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_MASTER)
    GmrBluetoothMission_MasterCommTask(elapsedMs);
#elif (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_SLAVE)
    GmrBluetoothMission_SlaveCommTask(elapsedMs);
#else
    (void)elapsedMs;
#endif
}

GmrBluetoothMissionResult GmrBluetoothMission_ControlPeriod(void)
{
    if ((g_active == 0U) ||
        (g_result != GMR_BLUETOOTH_MISSION_RESULT_RUNNING)) {
        return g_result;
    }
    if ((g_linkEstablished != 0U) &&
        (BluetoothService_IsConnected() == 0U)) {
        GmrBluetoothMission_SetError(
            GMR_BLUETOOTH_MISSION_ERROR_LINK_LOST);
        return g_result;
    }

#if (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_MASTER)
    GmrBluetoothMission_MasterControlPeriod();
#elif (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_SLAVE)
    GmrBluetoothMission_SlaveControlPeriod();
#endif
    return g_result;
}

void GmrBluetoothMission_HandleFastEvent(void)
{
#if (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_MASTER)
    if ((g_active != 0U) &&
        (g_phase == GMR_BLUETOOTH_MISSION_PHASE_RECORDING)) {
        MotorNoYaw_HandleFastEvent();
    }
#endif
}

void GmrBluetoothMission_GetStatus(GmrBluetoothMissionStatus *status)
{
    if (status == 0) {
        return;
    }
    status->active = g_active;
    status->connected = BluetoothService_IsConnected();
    status->role = (uint8_t)CAR_BLUETOOTH_ROLE;
    status->phase = g_phase;
    status->error = g_error;
    status->sampleCount = g_sampleCount;
    status->replayIndex = g_replayIndex;
    status->completedTurns = MotorNoYaw_GetTurnCount();
#if (CAR_BLUETOOTH_ROLE == CAR_BLUETOOTH_ROLE_SLAVE)
    status->replayLeftErrorCounts = g_replayLeftErrorCounts;
    status->replayRightErrorCounts = g_replayRightErrorCounts;
#else
    status->replayLeftErrorCounts = 0;
    status->replayRightErrorCounts = 0;
#endif
}

const char *GmrBluetoothMission_GetPhaseName(GmrBluetoothMissionPhase phase)
{
    switch (phase) {
    case GMR_BLUETOOTH_MISSION_PHASE_WAIT_LINK:
        return "Wait link";
    case GMR_BLUETOOTH_MISSION_PHASE_WAIT_READY:
        return "Wait ready";
    case GMR_BLUETOOTH_MISSION_PHASE_WAIT_START_ACK:
        return "Start ack";
    case GMR_BLUETOOTH_MISSION_PHASE_RECORDING:
        return "Recording";
    case GMR_BLUETOOTH_MISSION_PHASE_WAIT_OVER_ACK:
        return "Over ack";
    case GMR_BLUETOOTH_MISSION_PHASE_WAIT_START:
        return "Wait start";
    case GMR_BLUETOOTH_MISSION_PHASE_RECEIVING:
        return "Receiving";
    case GMR_BLUETOOTH_MISSION_PHASE_REPLAY:
        return "Replay";
    case GMR_BLUETOOTH_MISSION_PHASE_DONE:
        return "Done";
    case GMR_BLUETOOTH_MISSION_PHASE_ERROR:
        return "Error";
    case GMR_BLUETOOTH_MISSION_PHASE_IDLE:
    default:
        return "Idle";
    }
}

const char *GmrBluetoothMission_GetErrorName(GmrBluetoothMissionError error)
{
    switch (error) {
    case GMR_BLUETOOTH_MISSION_ERROR_DISABLED:
        return "BT disabled";
    case GMR_BLUETOOTH_MISSION_ERROR_LINK_LOST:
        return "Link lost";
    case GMR_BLUETOOTH_MISSION_ERROR_TX_QUEUE:
        return "TX queue";
    case GMR_BLUETOOTH_MISSION_ERROR_PROTOCOL:
        return "Protocol";
    case GMR_BLUETOOTH_MISSION_ERROR_SAMPLE_ORDER:
        return "RX order";
    case GMR_BLUETOOTH_MISSION_ERROR_BUFFER_FULL:
        return "Buffer full";
    case GMR_BLUETOOTH_MISSION_ERROR_DISTANCE_RANGE:
        return "Dist range";
    case GMR_BLUETOOTH_MISSION_ERROR_REPLAY_TIMEOUT:
        return "Dist timeout";
    case GMR_BLUETOOTH_MISSION_ERROR_LINE_FOLLOW:
        return "Line stop";
    case GMR_BLUETOOTH_MISSION_ERROR_NONE:
    default:
        return "None";
    }
}

#endif
