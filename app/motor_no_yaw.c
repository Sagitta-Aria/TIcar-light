#include "motor_no_yaw.h"

#include "board_config.h"
#include "gray.h"
#include "log_uart.h"
#include "motion.h"
#include "motor_enable.h"
#include "ti_msp_dl_config.h"

#define MOTOR_NO_YAW_S1_MASK        (0x40U)
#define MOTOR_NO_YAW_S2_MASK        (0x20U)
#define MOTOR_NO_YAW_S3_MASK        (0x10U)
#define MOTOR_NO_YAW_S4_MASK        (0x08U)
#define MOTOR_NO_YAW_S5_MASK        (0x04U)
#define MOTOR_NO_YAW_S6_MASK        (0x02U)
#define MOTOR_NO_YAW_S7_MASK        (0x01U)

/* 强转回归参考点：现在用 S4 重新压线作为回到普通循迹的条件。 */
#define MOTOR_NO_YAW_RETURN_MASK    MOTOR_NO_YAW_S4_MASK

#define MOTOR_NO_YAW_INVALID_TICKS  (0xFFFFU)

#if (CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US == 0U)
#error "CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US must be greater than 0"
#endif

#define MOTOR_NO_YAW_TIMER_DIV_TICKS \
    (((STEPPER_TIMER_TICK_HZ * CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US) + \
        999999U) / 1000000U)

#if (MOTOR_NO_YAW_TIMER_DIV_TICKS == 0U)
#error "MOTOR_NO_YAW_TIMER_DIV_TICKS must be greater than 0"
#endif

#define MOTOR_NO_YAW_RIGHT_TURN_WINDOW_SAMPLES \
    (((CAR_MOTOR_NO_YAW_RIGHT_TURN_WINDOW_MS * 1000U) + \
        CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US - 1U) / \
        CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US)

typedef struct {
    int16_t lineError;
    int16_t lastLeftSpeedSps;
    int16_t lastRightSpeedSps;
    uint16_t turnTicks;
    uint16_t lineLostTicks;
    volatile uint16_t s1RecentSamples;
    volatile uint16_t s2RecentSamples;
    uint8_t phase;
    volatile uint8_t digitalMask;
    uint8_t hasLastLineCommand;
    volatile uint8_t turnFlag;
    volatile uint8_t returnFlag;
    volatile uint8_t rightTurnRequest;
    volatile uint8_t returnLineRequest;
    volatile uint8_t running;
    uint8_t stopLogPending;
    const char *stopReason;
    volatile MotorNoYawState state;
} MotorNoYawControl;

static MotorNoYawControl g_motorNoYaw;

/* 作用：把毫秒换成 App_Task 的软件计时 tick。 */
static uint16_t MotorNoYaw_MsToTicks(uint16_t timeMs)
{
    return (uint16_t)((timeMs + CAR_APP_LOOP_DELAY_MS - 1U) /
        CAR_APP_LOOP_DELAY_MS);
}

/* 作用：把速度限制在普通循迹允许范围，普通循迹不允许单轮停车。 */
static int16_t MotorNoYaw_ClampLineSpeed(int32_t speed)
{
    if (speed < (int32_t)CAR_MOTOR_NO_YAW_MIN_LINE_SPEED_SPS) {
        return (int16_t)CAR_MOTOR_NO_YAW_MIN_LINE_SPEED_SPS;
    }
    if (speed > (int32_t)CAR_STEPPER_SPEED_MAX_SPS) {
        return (int16_t)CAR_STEPPER_SPEED_MAX_SPS;
    }
    return (int16_t)speed;
}

/* 作用：限制差速修正，防止普通循迹变成原地强转。 */
static int16_t MotorNoYaw_ClampCorrection(int32_t correction)
{
    if (correction > (int32_t)CAR_MOTOR_NO_YAW_TURN_LIMIT_SPS) {
        return (int16_t)CAR_MOTOR_NO_YAW_TURN_LIMIT_SPS;
    }
    if (correction < -(int32_t)CAR_MOTOR_NO_YAW_TURN_LIMIT_SPS) {
        return (int16_t)(-(int32_t)CAR_MOTOR_NO_YAW_TURN_LIMIT_SPS);
    }
    return (int16_t)correction;
}

/* 作用：吃掉很小的偏差，车在中间附近时不要来回抖。 */
static int16_t MotorNoYaw_ApplyLineDeadband(int16_t error)
{
    int32_t adjusted = error;

    if ((adjusted > -(int32_t)CAR_MOTOR_NO_YAW_LINE_DEADBAND) &&
        (adjusted < (int32_t)CAR_MOTOR_NO_YAW_LINE_DEADBAND)) {
        return 0;
    }
    if (adjusted > 0) {
        adjusted -= (int32_t)CAR_MOTOR_NO_YAW_LINE_DEADBAND;
    } else if (adjusted < 0) {
        adjusted += (int32_t)CAR_MOTOR_NO_YAW_LINE_DEADBAND;
    }

    return (int16_t)adjusted;
}

/* 作用：NO YAW 只读数字量，用最快的 GPIO 方式拿到 S1~S7 的 mask。 */
static void MotorNoYaw_UpdateGrayFast(void)
{
    g_motorNoYaw.digitalMask = Gray_ReadDigitalMaskFast();
}

/*
 * 作用：按数字量算一个很直白的偏差。
 * 说明：S1/S7 权重最大，S2/S6 次之，S3/S5 小修，S4 是中心。
 */
static uint8_t MotorNoYaw_CalcDigitalLineError(int16_t *error)
{
    static const int16_t weight[GRAY_SENSOR_COUNT] = {
        -9, -5, -2, 0, 2, 5, 9
    };
    int16_t sum = 0;
    uint8_t count = 0U;
    uint8_t mask = g_motorNoYaw.digitalMask;
    uint8_t bit;
    uint32_t i;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        bit = (uint8_t)(1U << ((GRAY_SENSOR_COUNT - 1U) - i));
        if ((mask & bit) != 0U) {
            sum = (int16_t)(sum + weight[i]);
            ++count;
        }
    }

    if (count == 0U) {
        if (error != 0) {
            *error = 0;
        }
        return 0U;
    }

    if (error != 0) {
        *error = (int16_t)(((int16_t)(sum / (int16_t)count)) *
            (int16_t)GRAY_LINE_ERROR_SCALE);
    }
    return 1U;
}

/* 作用：普通循迹输出差速，方向沿用原来的 NO YAW 公式。 */
static void MotorNoYaw_ApplyLineControl(void)
{
    int16_t error = 0;
    int16_t lineError;
    int16_t correction;
    int16_t baseSpeed = (int16_t)CAR_MOTOR_NO_YAW_BASE_SPEED_SPS;
    int16_t leftSpeed;
    int16_t rightSpeed;

    if (MotorNoYaw_CalcDigitalLineError(&error) != 0U) {
        g_motorNoYaw.lineError = error;
    }

    lineError = MotorNoYaw_ApplyLineDeadband(g_motorNoYaw.lineError);
    correction = MotorNoYaw_ClampCorrection(
        ((int32_t)lineError * (int32_t)CAR_MOTOR_NO_YAW_TURN_GAIN) /
            (int32_t)GRAY_LINE_ERROR_SCALE);

    leftSpeed = MotorNoYaw_ClampLineSpeed((int32_t)baseSpeed -
        (int32_t)correction);
    rightSpeed = MotorNoYaw_ClampLineSpeed((int32_t)baseSpeed +
        (int32_t)correction);

    g_motorNoYaw.lastLeftSpeedSps = leftSpeed;
    g_motorNoYaw.lastRightSpeedSps = rightSpeed;
    g_motorNoYaw.hasLastLineCommand = 1U;
    Motion_SetChassisCommand(leftSpeed, rightSpeed);
}

/* 作用：短时间丢线时先沿用上一拍，真丢线再温和搜线。 */
static void MotorNoYaw_ApplyLineLostCommand(void)
{
    if (g_motorNoYaw.hasLastLineCommand != 0U) {
        Motion_SetChassisCommand(g_motorNoYaw.lastLeftSpeedSps,
            g_motorNoYaw.lastRightSpeedSps);
        return;
    }

    Motion_SetChassisCommand(0,
        (int16_t)CAR_MOTOR_NO_YAW_DEFAULT_SEARCH_SPEED_SPS);
}

/* 作用：异常停车，OLED 保留在 NO YAW 页面，方便看 mask/err/state。 */
static void MotorNoYaw_StopWithReason(const char *reason)
{
    Motion_SetChassisCommand(0, 0);
    MotorEnable_SetChassis(CAR_STEPPER_ENABLE_DEFAULT_ON);
    g_motorNoYaw.running = 0U;
    g_motorNoYaw.stopReason = reason;
    g_motorNoYaw.stopLogPending = 1U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_STOP;
}

void MotorNoYaw_LogStopReason(void)
{
    if (g_motorNoYaw.stopLogPending == 0U) {
        return;
    }

    g_motorNoYaw.stopLogPending = 0U;
    LOG_RAW("[NO YAW] stop ");
    LOG_RAW(g_motorNoYaw.stopReason);
    LOG_RAW(" mask=");
    LogUart_SendHex32(g_motorNoYaw.digitalMask);
    LOG_RAW(" err=");
    LogUart_SendSigned(g_motorNoYaw.lineError);
    LOG_RAW(" phase=");
    LogUart_SendUnsigned(g_motorNoYaw.phase);
    LOG_LINE("");
}

/* 作用：记录 S1/S2 最近有没有灭灯，窗口内两个都出现就认为是右直角。 */
static uint8_t MotorNoYaw_RecordRightTurnSample(uint8_t mask)
{
    if ((mask & MOTOR_NO_YAW_S1_MASK) != 0U) {
        g_motorNoYaw.s1RecentSamples =
            (uint16_t)MOTOR_NO_YAW_RIGHT_TURN_WINDOW_SAMPLES;
    } else if (g_motorNoYaw.s1RecentSamples > 0U) {
        --g_motorNoYaw.s1RecentSamples;
    }

    if ((mask & MOTOR_NO_YAW_S2_MASK) != 0U) {
        g_motorNoYaw.s2RecentSamples =
            (uint16_t)MOTOR_NO_YAW_RIGHT_TURN_WINDOW_SAMPLES;
    } else if (g_motorNoYaw.s2RecentSamples > 0U) {
        --g_motorNoYaw.s2RecentSamples;
    }

    if ((g_motorNoYaw.s1RecentSamples > 0U) &&
        (g_motorNoYaw.s2RecentSamples > 0U)) {
        g_motorNoYaw.s1RecentSamples = 0U;
        g_motorNoYaw.s2RecentSamples = 0U;
        return 1U;
    }

    return 0U;
}

/* 作用：S1/S2 触发后，先让车头继续往直角里走一点。 */
static void MotorNoYaw_StartTurnApproach(void)
{
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.s1RecentSamples = 0U;
    g_motorNoYaw.s2RecentSamples = 0U;
    g_motorNoYaw.turnFlag = 0U;      /* 进弯后不再允许重复触发强转。 */
    g_motorNoYaw.returnFlag = 0U;    /* 前进阶段不判断回归点。 */
    g_motorNoYaw.rightTurnRequest = 0U;
    g_motorNoYaw.returnLineRequest = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_TURN_APPROACH;
}

/* 作用：前进时间到了以后进入强右转，右轮停，左轮用配置速度。 */
static void MotorNoYaw_StartRightTurn(void)
{
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.hasLastLineCommand = 0U;
    g_motorNoYaw.turnFlag = 0U;      /* 强转中不准再次进入强转。 */
    g_motorNoYaw.returnFlag = 1U;    /* 强转中只允许 S4 触发回循迹。 */
    g_motorNoYaw.rightTurnRequest = 0U;
    g_motorNoYaw.returnLineRequest = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_TURN_RIGHT;
}

/* 作用：S4 再次灭灯后，认为右直角转够了，回普通循迹。 */
static void MotorNoYaw_FinishRightTurn(void)
{
    g_motorNoYaw.phase = (uint8_t)((g_motorNoYaw.phase + 1U) & 0x03U);
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.hasLastLineCommand = 0U;
    g_motorNoYaw.s1RecentSamples = 0U;
    g_motorNoYaw.s2RecentSamples = 0U;
    g_motorNoYaw.turnFlag = 1U;      /* 回到正常循迹后，重新允许下一次强转。 */
    g_motorNoYaw.returnFlag = 0U;    /* 正常循迹中不判断 S4 回归。 */
    g_motorNoYaw.rightTurnRequest = 0U;
    g_motorNoYaw.returnLineRequest = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_LINE;
}

/*
 * 作用：TIMG0 中断里的灰度快采样。
 * 使用场景：每个 STEP 定时器 tick 调一次，本函数内部再分频到 100us 左右。
 * 说明：中断里只读 GPIO 和置请求标志，不直接控制电机、不打印、不刷屏。
 */
void MotorNoYaw_TimerSample(void)
{
    static uint16_t sampleDivTicks;
    uint8_t mask;
    MotorNoYawState state;

    if (g_motorNoYaw.running == 0U) {
        sampleDivTicks = 0U;
        return;
    }

    ++sampleDivTicks;
    if (sampleDivTicks < (uint16_t)MOTOR_NO_YAW_TIMER_DIV_TICKS) {
        return;
    }
    sampleDivTicks = 0U;

    mask = Gray_ReadDigitalMaskFast();
    g_motorNoYaw.digitalMask = mask;
    state = g_motorNoYaw.state;

    if (state == MOTOR_NO_YAW_STATE_LINE) {
        if ((g_motorNoYaw.turnFlag != 0U) &&
            (MotorNoYaw_RecordRightTurnSample(mask) != 0U)) {
            g_motorNoYaw.rightTurnRequest = 1U;
        }
        return;
    }

    if (state == MOTOR_NO_YAW_STATE_TURN_RIGHT) {
        if ((g_motorNoYaw.returnFlag != 0U) &&
            ((mask & MOTOR_NO_YAW_RETURN_MASK) != 0U)) {
            g_motorNoYaw.returnLineRequest = 1U;
        }
    }
}

/* 作用：正常循迹，每 1ms 左右读一次灰度数字量。 */
static void MotorNoYaw_TaskLine(void)
{
    uint16_t lineLostTimeoutTicks =
        MotorNoYaw_MsToTicks(CAR_MOTOR_NO_YAW_LINE_LOST_TIMEOUT_MS);

    MotorNoYaw_UpdateGrayFast();

    if (g_motorNoYaw.rightTurnRequest != 0U) {
        g_motorNoYaw.rightTurnRequest = 0U;
        MotorNoYaw_StartTurnApproach();
        return;
    }

    if (g_motorNoYaw.digitalMask == 0U) {
        MotorNoYaw_ApplyLineLostCommand();
        if (g_motorNoYaw.lineLostTicks >= lineLostTimeoutTicks) {
            MotorNoYaw_StopWithReason("line-lost");
            return;
        }
        ++g_motorNoYaw.lineLostTicks;
        return;
    }

    g_motorNoYaw.lineLostTicks = 0U;
    MotorNoYaw_ApplyLineControl();
}

/* 作用：右直角触发后，先前进一小段，再开始强右转。 */
static void MotorNoYaw_TaskTurnApproach(void)
{
    uint16_t approachTicks =
        MotorNoYaw_MsToTicks(CAR_MOTOR_NO_YAW_TURN_APPROACH_MS);

    MotorNoYaw_UpdateGrayFast();

    if (g_motorNoYaw.hasLastLineCommand != 0U) {
        Motion_SetChassisCommand(g_motorNoYaw.lastLeftSpeedSps,
            g_motorNoYaw.lastRightSpeedSps);
    } else {
        Motion_SetChassisCommand((int16_t)CAR_MOTOR_NO_YAW_BASE_SPEED_SPS,
            (int16_t)CAR_MOTOR_NO_YAW_BASE_SPEED_SPS);
    }

    if (g_motorNoYaw.turnTicks < MOTOR_NO_YAW_INVALID_TICKS) {
        ++g_motorNoYaw.turnTicks;
    }
    if (g_motorNoYaw.turnTicks >= approachTicks) {
        MotorNoYaw_StartRightTurn();
    }
}

/* 作用：强右转，只在 returnFlag 打开时才用 S4 回到正常循迹。 */
static void MotorNoYaw_TaskTurnRight(void)
{
    uint16_t timeoutTicks =
        MotorNoYaw_MsToTicks(CAR_MOTOR_NO_YAW_TURN_HOLD_MS);

    Motion_SetChassisCommand((int16_t)CAR_MOTOR_NO_YAW_TURN_SPEED_SPS, 0);
    MotorNoYaw_UpdateGrayFast();

    if (g_motorNoYaw.turnTicks < MOTOR_NO_YAW_INVALID_TICKS) {
        ++g_motorNoYaw.turnTicks;
    }

    if ((g_motorNoYaw.returnLineRequest != 0U) ||
        ((g_motorNoYaw.returnFlag != 0U) &&
        ((g_motorNoYaw.digitalMask & MOTOR_NO_YAW_RETURN_MASK) != 0U))) {
        g_motorNoYaw.returnLineRequest = 0U;
        MotorNoYaw_FinishRightTurn();
        return;
    }

    if (g_motorNoYaw.turnTicks >= timeoutTicks) {
        MotorNoYaw_StopWithReason("turn-timeout");
    }
}

static void MotorNoYaw_ResetControl(void)
{
    g_motorNoYaw.lineError = 0;
    g_motorNoYaw.lastLeftSpeedSps = 0;
    g_motorNoYaw.lastRightSpeedSps = 0;
    g_motorNoYaw.turnTicks = 0U;
    g_motorNoYaw.lineLostTicks = 0U;
    g_motorNoYaw.s1RecentSamples = 0U;
    g_motorNoYaw.s2RecentSamples = 0U;
    g_motorNoYaw.phase = 0U;
    g_motorNoYaw.digitalMask = 0U;
    g_motorNoYaw.hasLastLineCommand = 0U;
    g_motorNoYaw.turnFlag = 1U;
    g_motorNoYaw.returnFlag = 0U;
    g_motorNoYaw.rightTurnRequest = 0U;
    g_motorNoYaw.returnLineRequest = 0U;
    g_motorNoYaw.stopLogPending = 0U;
    g_motorNoYaw.stopReason = 0;
}

void MotorNoYaw_Init(void)
{
    MotorNoYaw_ResetControl();
    g_motorNoYaw.running = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_IDLE;
}

void MotorNoYaw_Start(void)
{
    MotorNoYaw_ResetControl();
    MotorNoYaw_UpdateGrayFast();
    g_motorNoYaw.running = 1U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_LINE;
    MotorEnable_SetChassis(1U);
}

void MotorNoYaw_Stop(void)
{
    if (g_motorNoYaw.state != MOTOR_NO_YAW_STATE_IDLE) {
        Motion_SetChassisCommand(0, 0);
        MotorEnable_SetChassis(CAR_STEPPER_ENABLE_DEFAULT_ON);
    }
    g_motorNoYaw.running = 0U;
    g_motorNoYaw.state = MOTOR_NO_YAW_STATE_IDLE;
}

void MotorNoYaw_Task(void)
{
    if (g_motorNoYaw.running == 0U) {
        return;
    }

    switch (g_motorNoYaw.state) {
    case MOTOR_NO_YAW_STATE_LINE:
        MotorNoYaw_TaskLine();
        break;
    case MOTOR_NO_YAW_STATE_TURN_APPROACH:
        MotorNoYaw_TaskTurnApproach();
        break;
    case MOTOR_NO_YAW_STATE_TURN_RIGHT:
        MotorNoYaw_TaskTurnRight();
        break;
    case MOTOR_NO_YAW_STATE_TURN_LEFT:
    case MOTOR_NO_YAW_STATE_STOP:
    case MOTOR_NO_YAW_STATE_IDLE:
    default:
        break;
    }
}

uint8_t MotorNoYaw_IsRunning(void)
{
    return g_motorNoYaw.running;
}

MotorNoYawState MotorNoYaw_GetState(void)
{
    return g_motorNoYaw.state;
}

const char *MotorNoYaw_GetStateName(void)
{
    switch (g_motorNoYaw.state) {
    case MOTOR_NO_YAW_STATE_LINE:
        return "Line";
    case MOTOR_NO_YAW_STATE_TURN_APPROACH:
        return "Approach";
    case MOTOR_NO_YAW_STATE_TURN_RIGHT:
        return "RTurn";
    case MOTOR_NO_YAW_STATE_TURN_LEFT:
        return "LTurn";
    case MOTOR_NO_YAW_STATE_STOP:
        return "Stop";
    case MOTOR_NO_YAW_STATE_IDLE:
    default:
        return "Idle";
    }
}

uint8_t MotorNoYaw_GetPhase(void)
{
    return g_motorNoYaw.phase;
}

uint8_t MotorNoYaw_GetDigitalMask(void)
{
    return g_motorNoYaw.digitalMask;
}

int16_t MotorNoYaw_GetLineError(void)
{
    return g_motorNoYaw.lineError;
}
