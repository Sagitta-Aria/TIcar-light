#include "state_machine.h"

#include "board_config.h"
#include "control_config.h"
#include "encoder_motor.h"
#include "gimbal.h"
#include "motion.h"
#include "motor.h"
#include "motor_enable.h"
#include "motor_no_yaw.h"
#include "rtos_app.h"
#include "staticconfig.h"
#include "tuning_console.h"
#include "vision.h"

static CarState g_carState = CAR_STATE_INIT;
static uint8_t g_missionId;
static uint8_t g_mission1LapCount = 1U;
static uint8_t g_mission3Distance;
static CarChassisDriveMode g_mission6DriveMode;
static uint16_t g_mission6DriveSpeed;
static CarChassisDriveMode g_mission7DriveMode;
static uint16_t g_mission7DriveSpeed;
static int32_t g_mission7RightEncoderBase;
static CarMission4Stage g_mission4Stage = CAR_MISSION4_STAGE_IDLE;
static uint32_t g_mission4Flag;
static uint32_t g_mission4LastVisionFrame;
static int32_t g_mission4ExtraLeftBase;
static int32_t g_mission4ExtraRightBase;
static int32_t g_mission4YawTurnBase;
static uint8_t g_mission4StableFrames;
static uint8_t g_mission4ExtraActive;
static uint8_t g_mission4YawTurnActive;
static uint8_t g_missionGimbalPrepId;
static uint16_t g_missionGimbalPrepWaitTicks;
static int32_t g_missionGimbalPrepYawBase;
static MotorNoYawState g_mission4LastNoYawState = MOTOR_NO_YAW_STATE_IDLE;

typedef enum {
    MISSION_GIMBAL_PREP_IDLE = 0,
    MISSION_GIMBAL_PREP_YAW,
    MISSION_GIMBAL_PREP_WAIT,
    MISSION_GIMBAL_PREP_DONE
} MissionGimbalPrepStage;

static MissionGimbalPrepStage g_missionGimbalPrepStage;

#define MISSION4_LOCK_STABLE_FRAMES     (5U)
#define MISSION4_LOCK_EXTRA_DEADBAND    (20U)
#define MISSION4_LAP_TURNS              (4U)
#define MISSION4_EXTRA_ENCODER_COUNTS   (12000U)
#define MISSION_GIMBAL_PREP_YAW_STEPS   (2000U)
#define MISSION_GIMBAL_PREP_YAW_SPEED_SPS (5000U)
#define MISSION_GIMBAL_PREP_WAIT_MS     (500U)
#define MISSION4_GIMBAL_YAW_RATIO_SCALE (1000L)
#define MISSION_GIMBAL_PREP_WAIT_TICKS \
    ((MISSION_GIMBAL_PREP_WAIT_MS + CAR_APP_LOOP_DELAY_MS - 1U) / \
        CAR_APP_LOOP_DELAY_MS)

typedef struct {
    uint32_t yawSteps;
    int16_t yawRatioX1000;
    uint16_t yawSpeedSps;
    uint16_t yawAccelStepSps;
    uint16_t yawDecelStepSps;
} Mission4YawConfig;

static const Mission4YawConfig g_mission4YawConfigs[3] = {
    {
        (uint32_t)CAR_MISSION4_GIMBAL_NEAR_YAW_STEPS,
        (int16_t)CAR_MISSION4_GIMBAL_NEAR_YAW_RATIO_X1000,
        (uint16_t)CAR_MISSION4_GIMBAL_NEAR_YAW_SPEED_SPS,
        (uint16_t)CAR_MISSION4_GIMBAL_NEAR_YAW_ACCEL_STEP_SPS,
        (uint16_t)CAR_MISSION4_GIMBAL_NEAR_YAW_DECEL_STEP_SPS
    },
    {
        (uint32_t)CAR_MISSION4_GIMBAL_MID_YAW_STEPS,
        (int16_t)CAR_MISSION4_GIMBAL_MID_YAW_RATIO_X1000,
        (uint16_t)CAR_MISSION4_GIMBAL_MID_YAW_SPEED_SPS,
        (uint16_t)CAR_MISSION4_GIMBAL_MID_YAW_ACCEL_STEP_SPS,
        (uint16_t)CAR_MISSION4_GIMBAL_MID_YAW_DECEL_STEP_SPS
    },
    {
        (uint32_t)CAR_MISSION4_GIMBAL_FAR_YAW_STEPS,
        (int16_t)CAR_MISSION4_GIMBAL_FAR_YAW_RATIO_X1000,
        (uint16_t)CAR_MISSION4_GIMBAL_FAR_YAW_SPEED_SPS,
        (uint16_t)CAR_MISSION4_GIMBAL_FAR_YAW_ACCEL_STEP_SPS,
        (uint16_t)CAR_MISSION4_GIMBAL_FAR_YAW_DECEL_STEP_SPS
    }
};

static void StateMachine_Enter(CarState nextState);

/* 作用：把菜单任务事件转换成 1~7 的任务编号。 */
static uint8_t StateMachine_GetMissionIdFromEvent(CarEvent event)
{
    switch (event) {
    case CAR_EVENT_MISSION_1_START:
        return 1U;
    case CAR_EVENT_MISSION_2_START:
        return 2U;
    case CAR_EVENT_MISSION_3_START:
        return 3U;
    case CAR_EVENT_MISSION_4_START:
        return 4U;
    case CAR_EVENT_MISSION_5_START:
        return 5U;
    case CAR_EVENT_MISSION_6_START:
        return 6U;
    case CAR_EVENT_MISSION_7_START:
        return 7U;
    default:
        return 0U;
    }
}

static uint16_t StateMachine_Abs16(int16_t value)
{
    return (value < 0) ? (uint16_t)(-(int32_t)value) : (uint16_t)value;
}

static uint32_t StateMachine_AbsStepDelta(int32_t now, int32_t base)
{
    return (now >= base) ? (uint32_t)(now - base) :
        (uint32_t)(base - now);
}

/* 作用：Task4 一圈后的延长段，用左右编码器count绝对增量平均值计距离。 */
static uint32_t StateMachine_GetMission4ExtraEncoderCounts(void)
{
    uint32_t leftCounts = StateMachine_AbsStepDelta(
        Motor_GetStepCount(MOTOR_CHASSIS_LEFT), g_mission4ExtraLeftBase);
    uint32_t rightCounts = StateMachine_AbsStepDelta(
        Motor_GetStepCount(MOTOR_CHASSIS_RIGHT), g_mission4ExtraRightBase);

    return (leftCounts + rightCounts) / 2U;
}

/* 作用：Task4 用 flag 判断当前位置，四个边循环使用近/中/远/中参数。 */
static StaticConfigDistance StateMachine_GetMission4Distance(uint32_t flag)
{
    static const StaticConfigDistance distanceByPhase[4] = {
        STATICCONFIG_DISTANCE_NEAR,
        STATICCONFIG_DISTANCE_MID,
        STATICCONFIG_DISTANCE_FAR,
        STATICCONFIG_DISTANCE_MID
    };

    return distanceByPhase[(uint8_t)(flag & 0x03U)];
}

static const Mission4YawConfig *StateMachine_GetMission4YawConfig(uint32_t flag)
{
    return &g_mission4YawConfigs[
        (uint8_t)StateMachine_GetMission4Distance(flag)];
}

/* 作用：Task4 始终追中心点，只按当前位置切换近/中/远参数。 */
static void StateMachine_ApplyMission4GimbalConfig(uint32_t flag)
{
    StaticConfig_SetActiveByDistanceMode(
        StateMachine_GetMission4Distance(flag),
        STATICCONFIG_MODE_CENTER);
}

static void StateMachine_ResetMission4(void)
{
    g_mission4Stage = CAR_MISSION4_STAGE_IDLE;
    g_mission4Flag = 0U;
    g_mission4LastVisionFrame = 0U;
    g_mission4ExtraLeftBase = 0;
    g_mission4ExtraRightBase = 0;
    g_mission4YawTurnBase = 0;
    g_mission4StableFrames = 0U;
    g_mission4ExtraActive = 0U;
    g_mission4YawTurnActive = 0U;
    g_mission4LastNoYawState = MOTOR_NO_YAW_STATE_IDLE;
    Gimbal_SetYawFeedForward(0);
}

static void StateMachine_ResetMissionGimbalPrep(void)
{
    g_missionGimbalPrepId = 0U;
    g_missionGimbalPrepWaitTicks = 0U;
    g_missionGimbalPrepYawBase = 0;
    g_missionGimbalPrepStage = MISSION_GIMBAL_PREP_IDLE;
}

/* 作用：停止所有可能输出运动命令的正式模块。 */
static void StateMachine_StopRuntimeModules(void)
{
    TuningConsole_Stop();
    MotorNoYaw_Stop();
    Vision_Stop();
    Gimbal_SetEnabled(0U);
    Gimbal_SetYawFeedForward(0);
    MotorEnable_SetGimbal(CAR_GIMBAL_ENABLE_DEFAULT_ON);
    Motion_Stop();
    StateMachine_ResetMission4();
    StateMachine_ResetMissionGimbalPrep();
}

/* 作用：启动一段 5000 step yaw 搜索。 */
static void StateMachine_StartMissionGimbalYawSegment(void)
{
    g_missionGimbalPrepWaitTicks = 0U;
    g_missionGimbalPrepYawBase = Motor_GetStepCount(MOTOR_GIMBAL_1);
    g_missionGimbalPrepStage = MISSION_GIMBAL_PREP_YAW;

    Motor_Set(MOTOR_GIMBAL_1, MOTOR_FORWARD,
        (uint16_t)MISSION_GIMBAL_PREP_YAW_SPEED_SPS);
}

/* 作用：Task3/Task4 正式追踪前，先静止等视觉；没找到再循环 yaw 搜索目标。 */
static void StateMachine_StartMissionGimbalPrep(uint8_t missionId)
{
    g_missionGimbalPrepId = missionId;

    Vision_Start();
    Gimbal_SetEnabled(0U);
    MotorEnable_SetGimbal(1U);

    if ((missionId == 3U) || (missionId == 4U)) {
        g_missionGimbalPrepWaitTicks = 0U;
        g_missionGimbalPrepStage = MISSION_GIMBAL_PREP_WAIT;
    } else {
        StateMachine_StartMissionGimbalYawSegment();
    }
}

static uint8_t StateMachine_ClampMission1LapCount(uint8_t lapCount)
{
    if (lapCount < 1U) {
        return 1U;
    }
    if (lapCount > 5U) {
        return 5U;
    }
    return lapCount;
}

static uint8_t StateMachine_ClampMission3Distance(uint8_t distance)
{
    if (distance > (uint8_t)STATICCONFIG_DISTANCE_FAR) {
        return (uint8_t)STATICCONFIG_DISTANCE_NEAR;
    }
    return distance;
}

static uint16_t StateMachine_ClampMissionDriveSpeed(uint16_t speed)
{
    if (speed < (uint16_t)CHASSIS_DEBUG_SPEED_MIN) {
        return (uint16_t)CHASSIS_DEBUG_SPEED_MIN;
    }
    if (speed > (uint16_t)CHASSIS_DEBUG_SPEED_MAX) {
        return (uint16_t)CHASSIS_DEBUG_SPEED_MAX;
    }
    return speed;
}

static int16_t StateMachine_DrivePercentToPwm(uint16_t percent)
{
    return (int16_t)(((uint32_t)CHASSIS_PWM_LIMIT_COUNTS * percent) /
        100U);
}

static uint8_t StateMachine_IsMission4Locked(void)
{
    const StaticConfigGimbalTask *config = StaticConfig_GetActiveGimbal();
    uint32_t limitX = (uint32_t)config->deadbandX +
        (uint32_t)MISSION4_LOCK_EXTRA_DEADBAND;
    uint32_t limitY = (uint32_t)config->deadbandY +
        (uint32_t)MISSION4_LOCK_EXTRA_DEADBAND;

    if (Vision_HasFrame() == 0U) {
        return 0U;
    }

    return (uint8_t)(((uint32_t)StateMachine_Abs16(Gimbal_GetErrorX()) <=
        limitX) &&
        ((uint32_t)StateMachine_Abs16(Gimbal_GetErrorY()) <= limitY));
}

static void StateMachine_StartMission4Line(void)
{
    g_mission4Flag = 0U;
    g_mission4ExtraActive = 0U;
    g_mission4YawTurnActive = 0U;
    StateMachine_ApplyMission4GimbalConfig(g_mission4Flag);
    MotorNoYaw_StartMission4();
    g_mission4LastNoYawState = MotorNoYaw_GetState();
    g_mission4Stage = CAR_MISSION4_STAGE_LINE;
}

static uint16_t StateMachine_AbsMotorCommand(int16_t command)
{
    return (command < 0) ? (uint16_t)(-(int32_t)command) : (uint16_t)command;
}

static uint16_t StateMachine_GetMission4ChassisSpeed(void)
{
    uint32_t leftSpeed = StateMachine_AbsMotorCommand(
        Motor_GetCommand(MOTOR_CHASSIS_LEFT));
    uint32_t rightSpeed = StateMachine_AbsMotorCommand(
        Motor_GetCommand(MOTOR_CHASSIS_RIGHT));
    uint32_t avgSpeed = (leftSpeed + rightSpeed) / 2U;

    if (avgSpeed > (uint32_t)CHASSIS_TARGET_LIMIT_CPS) {
        return (uint16_t)CHASSIS_TARGET_LIMIT_CPS;
    }
    return (uint16_t)avgSpeed;
}

static int16_t StateMachine_ClampMission4YawCommand(int32_t command)
{
    if (command > (int32_t)CAR_STEPPER_SPEED_MAX_SPS) {
        return (int16_t)CAR_STEPPER_SPEED_MAX_SPS;
    }
    if (command < -(int32_t)CAR_STEPPER_SPEED_MAX_SPS) {
        return (int16_t)(-(int32_t)CAR_STEPPER_SPEED_MAX_SPS);
    }
    return (int16_t)command;
}

static int16_t StateMachine_GetMission4YawBaseCommand(void)
{
    const Mission4YawConfig *config =
        StateMachine_GetMission4YawConfig(g_mission4Flag);
    int32_t command = ((int32_t)StateMachine_GetMission4ChassisSpeed() *
        (int32_t)config->yawRatioX1000) /
        MISSION4_GIMBAL_YAW_RATIO_SCALE;

    return StateMachine_ClampMission4YawCommand(command);
}

static int16_t StateMachine_GetMission4YawTurnCommand(int16_t baseCommand)
{
    const Mission4YawConfig *config =
        StateMachine_GetMission4YawConfig(g_mission4Flag);
    int32_t command = (int32_t)config->yawSpeedSps;

    if (baseCommand < 0) {
        command = -command;
    }
    return StateMachine_ClampMission4YawCommand(command);
}

static void StateMachine_SetMission4YawTurnRamp(uint8_t enabled)
{
    const Mission4YawConfig *config =
        StateMachine_GetMission4YawConfig(g_mission4Flag);

    if (enabled != 0U) {
        Motor_SetRampStep(MOTOR_GIMBAL_1,
            config->yawAccelStepSps,
            config->yawDecelStepSps);
    } else {
        Gimbal_ResetRamp();
    }
}

static uint8_t StateMachine_IsMission4TurnState(MotorNoYawState state)
{
    return (uint8_t)((state == MOTOR_NO_YAW_STATE_TURN_RIGHT) ? 1U : 0U);
}

static void StateMachine_UpdateMission4YawFeedForward(void)
{
    MotorNoYawState noYawState = MotorNoYaw_GetState();
    int16_t baseCommand = StateMachine_GetMission4YawBaseCommand();
    int16_t yawCommand = baseCommand;

    if ((StateMachine_IsMission4TurnState(noYawState) != 0U) &&
        (StateMachine_IsMission4TurnState(g_mission4LastNoYawState) == 0U)) {
        g_mission4YawTurnBase = Motor_GetStepCount(MOTOR_GIMBAL_1);
        g_mission4YawTurnActive = 1U;
        StateMachine_SetMission4YawTurnRamp(1U);
    } else if (StateMachine_IsMission4TurnState(noYawState) == 0U) {
        if (g_mission4YawTurnActive != 0U) {
            g_mission4YawTurnActive = 0U;
            StateMachine_SetMission4YawTurnRamp(0U);
        }
    }

    if (g_mission4YawTurnActive != 0U) {
        if (StateMachine_AbsStepDelta(Motor_GetStepCount(MOTOR_GIMBAL_1),
            g_mission4YawTurnBase) < (uint32_t)
            StateMachine_GetMission4YawConfig(g_mission4Flag)->yawSteps) {
            yawCommand = StateMachine_GetMission4YawTurnCommand(baseCommand);
        } else {
            g_mission4YawTurnActive = 0U;
            StateMachine_SetMission4YawTurnRamp(0U);
        }
    }

    Gimbal_SetYawFeedForward(yawCommand);
    g_mission4LastNoYawState = noYawState;
}

/* 作用：搜到 K230 坐标后，真正进入视觉云台闭环。 */
static void StateMachine_StartMissionGimbalTrack(void)
{
    Gimbal_SetTarget(0, 0);
    Gimbal_SetEnabled(1U);

    if (g_missionGimbalPrepId == 4U) {
        g_mission4LastVisionFrame = Vision_GetFrameCount();
        g_mission4StableFrames = 0U;
        g_mission4Stage = CAR_MISSION4_STAGE_TRACK;
    }

    g_missionGimbalPrepStage = MISSION_GIMBAL_PREP_DONE;
}

/* 作用：Task3/Task4 先静止识别 500ms；之后没有坐标就继续下一段 yaw。 */
static void StateMachine_TaskMissionGimbalPrep(void)
{
    uint32_t yawSteps;

    if (g_missionGimbalPrepStage == MISSION_GIMBAL_PREP_YAW) {
        yawSteps = StateMachine_AbsStepDelta(
            Motor_GetStepCount(MOTOR_GIMBAL_1),
            g_missionGimbalPrepYawBase);
        if (yawSteps < (uint32_t)MISSION_GIMBAL_PREP_YAW_STEPS) {
            return;
        }

        Motor_Set(MOTOR_GIMBAL_1, MOTOR_COAST, 0U);
        g_missionGimbalPrepWaitTicks = 0U;
        g_missionGimbalPrepStage = MISSION_GIMBAL_PREP_WAIT;
        return;
    }

    if (g_missionGimbalPrepStage == MISSION_GIMBAL_PREP_WAIT) {
        if (Vision_HasFrame() != 0U) {
            StateMachine_StartMissionGimbalTrack();
            return;
        }

        if (g_missionGimbalPrepWaitTicks <
            (uint16_t)MISSION_GIMBAL_PREP_WAIT_TICKS) {
            ++g_missionGimbalPrepWaitTicks;
            return;
        }

        StateMachine_StartMissionGimbalYawSegment();
    }
}

/* 作用：Task4 先等云台连续锁住几帧，再启动 NO YAW 循迹。 */
static void StateMachine_TaskMission4Track(void)
{
    uint32_t frameCount = Vision_GetFrameCount();

    if (frameCount == g_mission4LastVisionFrame) {
        return;
    }
    g_mission4LastVisionFrame = frameCount;

    if (StateMachine_IsMission4Locked() != 0U) {
        if (g_mission4StableFrames < MISSION4_LOCK_STABLE_FRAMES) {
            ++g_mission4StableFrames;
        }
    } else {
        g_mission4StableFrames = 0U;
    }

    if (g_mission4StableFrames >= MISSION4_LOCK_STABLE_FRAMES) {
        StateMachine_StartMission4Line();
    }
}

/* 作用：Task4 循迹时用已完成转向次数当 flag，按 flag 切云台参数。 */
static void StateMachine_TaskMission4Line(void)
{
    uint32_t flag;

    MotorNoYaw_Task();
    if (MotorNoYaw_IsRunning() == 0U) {
        Gimbal_SetYawFeedForward(0);
        if (MotorNoYaw_GetState() == MOTOR_NO_YAW_STATE_STOP) {
            StateMachine_Enter(CAR_STATE_STOP);
        }
        return;
    }
    StateMachine_UpdateMission4YawFeedForward();

    flag = MotorNoYaw_GetTurnCount();
    if (flag != g_mission4Flag) {
        g_mission4Flag = flag;
        StateMachine_ApplyMission4GimbalConfig(g_mission4Flag);
    }

    if ((g_mission4ExtraActive == 0U) &&
        (flag >= (uint32_t)MISSION4_LAP_TURNS)) {
        g_mission4ExtraLeftBase = Motor_GetStepCount(MOTOR_CHASSIS_LEFT);
        g_mission4ExtraRightBase = Motor_GetStepCount(MOTOR_CHASSIS_RIGHT);
        g_mission4ExtraActive = 1U;
        return;
    }

    if ((g_mission4ExtraActive != 0U) &&
        (StateMachine_GetMission4ExtraEncoderCounts() >=
            (uint32_t)MISSION4_EXTRA_ENCODER_COUNTS)) {
        StateMachine_Enter(CAR_STATE_FINISHED);
    }
}

static void StateMachine_TaskMission4(void)
{
    if (g_missionGimbalPrepStage != MISSION_GIMBAL_PREP_DONE) {
        StateMachine_TaskMissionGimbalPrep();
        return;
    }

    if (g_mission4Stage == CAR_MISSION4_STAGE_TRACK) {
        StateMachine_TaskMission4Track();
    }
}

/* 作用：进入任务状态时启动对应正式任务。 */
static void StateMachine_EnterMission(void)
{
    StateMachine_StopRuntimeModules();

    if (g_missionId == 1U) {
        MotorNoYaw_Start();
    } else if (g_missionId == 2U) {
        /* Task2：固定使用第 1 套云台参数，直接开启视觉输入和云台闭环。 */
        StaticConfig_SetActiveTask(STATICCONFIG_TASK_NEAR_CENTER);
        Vision_Start();
        MotorEnable_SetGimbal(1U);
        Gimbal_SetTarget(0, 0);
        Gimbal_SetEnabled(1U);
    } else if (g_missionId == 3U) {
        /* Task3：按菜单选择近/中/远，三档都使用中心点误差追踪。 */
        StaticConfig_SetActiveByDistanceMode(
            (StaticConfigDistance)g_mission3Distance,
            STATICCONFIG_MODE_CENTER);
        StateMachine_StartMissionGimbalPrep(3U);
    } else if (g_missionId == 4U) {
        /* Task4：先做 yaw 前置，再按中心点模式抓点，稳定后启动 NO YAW。 */
        StateMachine_ResetMission4();
        StateMachine_ApplyMission4GimbalConfig(0U);
        StateMachine_StartMissionGimbalPrep(4U);
    } else if (g_missionId == 5U) {
        /* Task5 owns the chassis through the UART calibration console. */
        TuningConsole_Start();
    } else if (g_missionId == 6U) {
        if (g_mission6DriveMode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) {
            Motion_SetChassisPeriodCommand((int16_t)g_mission6DriveSpeed,
                (int16_t)g_mission6DriveSpeed);
        } else {
            int16_t pwm = StateMachine_DrivePercentToPwm(
                g_mission6DriveSpeed);
            EncoderMotor_SetOpenLoopPwm(pwm, pwm);
        }
    } else if (g_missionId == 7U) {
        /* Task7 counts only the right-wheel movement made after this start. */
        g_mission7RightEncoderBase =
            Motor_GetStepCount(MOTOR_CHASSIS_RIGHT);
        if (g_mission7DriveMode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) {
            Motion_SetChassisPeriodCommand(0,
                (int16_t)g_mission7DriveSpeed);
        } else {
            EncoderMotor_SetOpenLoopPwm(0,
                StateMachine_DrivePercentToPwm(g_mission7DriveSpeed));
        }
    }
}

/* 作用：统一进入新状态，保证离开任务时先停车。 */
static void StateMachine_Enter(CarState nextState)
{
    if (g_carState == nextState) {
        return;
    }

    if ((nextState == CAR_STATE_MENU) || (nextState == CAR_STATE_STOP) ||
        (nextState == CAR_STATE_FINISHED) || (nextState == CAR_STATE_ERROR)) {
        StateMachine_StopRuntimeModules();
    }

    g_carState = nextState;
    if (g_carState == CAR_STATE_MENU) {
        g_missionId = 0U;
    } else if (g_carState == CAR_STATE_MISSION) {
        StateMachine_EnterMission();
    }
    RtosApp_NotifyMission();
    RtosApp_NotifyUi();
}

void StateMachine_Init(void)
{
    g_carState = CAR_STATE_INIT;
    g_missionId = 0U;
    g_mission3Distance = (uint8_t)STATICCONFIG_DISTANCE_NEAR;
    g_mission6DriveMode = (CHASSIS_TASK6_DEFAULT_CLOSED_LOOP != 0U) ?
        CAR_CHASSIS_DRIVE_CLOSED_LOOP : CAR_CHASSIS_DRIVE_OPEN_LOOP;
    g_mission6DriveSpeed = (g_mission6DriveMode ==
        CAR_CHASSIS_DRIVE_CLOSED_LOOP) ?
        (uint16_t)CHASSIS_TASK6_CLOSED_SPEED_DEFAULT :
        (uint16_t)CHASSIS_TASK6_OPEN_SPEED_DEFAULT;
    g_mission7DriveMode = (CHASSIS_TASK7_DEFAULT_CLOSED_LOOP != 0U) ?
        CAR_CHASSIS_DRIVE_CLOSED_LOOP : CAR_CHASSIS_DRIVE_OPEN_LOOP;
    g_mission7DriveSpeed = (g_mission7DriveMode ==
        CAR_CHASSIS_DRIVE_CLOSED_LOOP) ?
        (uint16_t)CHASSIS_TASK7_CLOSED_SPEED_DEFAULT :
        (uint16_t)CHASSIS_TASK7_OPEN_SPEED_DEFAULT;
    g_mission7RightEncoderBase = 0;
    StateMachine_Enter(CAR_STATE_MENU);
}

void StateMachine_Dispatch(CarEvent event)
{
    uint8_t missionId;

    if (event == CAR_EVENT_NONE) {
        return;
    }

    if (event == CAR_EVENT_ERROR) {
        StateMachine_Enter(CAR_STATE_ERROR);
        return;
    }
    if (event == CAR_EVENT_STOP) {
        StateMachine_Enter(CAR_STATE_STOP);
        return;
    }
    if ((event == CAR_EVENT_MENU) || (event == CAR_EVENT_CLEAR_ERROR)) {
        StateMachine_Enter(CAR_STATE_MENU);
        return;
    }
    if (event == CAR_EVENT_FINISHED) {
        StateMachine_Enter(CAR_STATE_FINISHED);
        return;
    }

    if (g_carState == CAR_STATE_MENU) {
        missionId = StateMachine_GetMissionIdFromEvent(event);
        if (missionId != 0U) {
            g_missionId = missionId;
            StateMachine_Enter(CAR_STATE_MISSION);
        }
    }
}

void StateMachine_Task(void)
{
    switch (g_carState) {
    case CAR_STATE_MISSION:
        if (g_missionId == 3U) {
            if (g_missionGimbalPrepStage != MISSION_GIMBAL_PREP_DONE) {
                StateMachine_TaskMissionGimbalPrep();
            }
            break;
        }

        if (g_missionId == 4U) {
            StateMachine_TaskMission4();
            break;
        }

        if (g_missionId == 7U) {
            if (StateMachine_GetMission7EncoderCounts() >=
                (uint32_t)CHASSIS_TASK7_ENCODER_TARGET_COUNTS) {
                StateMachine_Enter(CAR_STATE_FINISHED);
            }
            break;
        }

        break;
    case CAR_STATE_INIT:
    case CAR_STATE_MENU:
    case CAR_STATE_FINISHED:
    case CAR_STATE_STOP:
    case CAR_STATE_ERROR:
    default:
        break;
    }
}

void StateMachine_ChassisControlPeriod(void)
{
    uint32_t targetTurns;

    if (g_carState != CAR_STATE_MISSION) {
        return;
    }

    if ((g_missionId == 4U) &&
        (g_mission4Stage == CAR_MISSION4_STAGE_LINE)) {
        StateMachine_TaskMission4Line();
        return;
    }

    if (g_missionId == 5U) {
        TuningConsole_ChassisControlPeriod();
        return;
    }

    if (g_missionId != 1U) {
        return;
    }

    MotorNoYaw_Task();
    if (MotorNoYaw_IsRunning() == 0U) {
        if (MotorNoYaw_GetState() == MOTOR_NO_YAW_STATE_STOP) {
            StateMachine_Enter(CAR_STATE_STOP);
        }
        return;
    }

    targetTurns = (uint32_t)g_mission1LapCount * 4U;
    if (MotorNoYaw_GetTurnCount() >= targetTurns) {
        StateMachine_Enter(CAR_STATE_FINISHED);
    }
}

void StateMachine_HandleChassisFastEvent(void)
{
    if (g_carState != CAR_STATE_MISSION) {
        return;
    }

    if (g_missionId == 1U) {
        MotorNoYaw_HandleFastEvent();
    } else if ((g_missionId == 4U) &&
        (g_mission4Stage == CAR_MISSION4_STAGE_LINE)) {
        MotorNoYaw_HandleFastEvent();
        StateMachine_UpdateMission4YawFeedForward();
    }
}

CarState StateMachine_GetState(void)
{
    return g_carState;
}

const char *StateMachine_GetStateName(CarState state)
{
    switch (state) {
    case CAR_STATE_INIT:
        return "init";
    case CAR_STATE_MENU:
        return "menu";
    case CAR_STATE_MISSION:
        return "mission";
    case CAR_STATE_FINISHED:
        return "finished";
    case CAR_STATE_STOP:
        return "stop";
    case CAR_STATE_ERROR:
        return "error";
    default:
        return "unknown";
    }
}

uint8_t StateMachine_GetMissionId(void)
{
    return g_missionId;
}

void StateMachine_SetMission1LapCount(uint8_t lapCount)
{
    g_mission1LapCount = StateMachine_ClampMission1LapCount(lapCount);
}

uint8_t StateMachine_GetMission1LapCount(void)
{
    return g_mission1LapCount;
}

void StateMachine_SetMission3Distance(uint8_t distance)
{
    g_mission3Distance = StateMachine_ClampMission3Distance(distance);
}

uint8_t StateMachine_GetMission3Distance(void)
{
    return g_mission3Distance;
}

void StateMachine_SetMissionDriveConfig(uint8_t missionId,
    CarChassisDriveMode mode, uint16_t speed)
{
    mode = (mode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ?
        CAR_CHASSIS_DRIVE_CLOSED_LOOP : CAR_CHASSIS_DRIVE_OPEN_LOOP;
    speed = StateMachine_ClampMissionDriveSpeed(speed);
    if (missionId == 6U) {
        g_mission6DriveMode = mode;
        g_mission6DriveSpeed = speed;
    } else if (missionId == 7U) {
        g_mission7DriveMode = mode;
        g_mission7DriveSpeed = speed;
    }
}

CarChassisDriveMode StateMachine_GetMissionDriveMode(uint8_t missionId)
{
    if (missionId == 6U) {
        return g_mission6DriveMode;
    }
    if (missionId == 7U) {
        return g_mission7DriveMode;
    }
    return CAR_CHASSIS_DRIVE_OPEN_LOOP;
}

uint16_t StateMachine_GetMissionDriveSpeed(uint8_t missionId)
{
    if (missionId == 6U) {
        return g_mission6DriveSpeed;
    }
    if (missionId == 7U) {
        return g_mission7DriveSpeed;
    }
    return 0U;
}

uint32_t StateMachine_GetMission7EncoderCounts(void)
{
    return StateMachine_AbsStepDelta(
        Motor_GetStepCount(MOTOR_CHASSIS_RIGHT),
        g_mission7RightEncoderBase);
}

CarMission4Stage StateMachine_GetMission4Stage(void)
{
    return g_mission4Stage;
}

uint32_t StateMachine_GetMission4Flag(void)
{
    return g_mission4Flag;
}

uint8_t StateMachine_IsMissionGimbalPrepDone(void)
{
    return (uint8_t)(g_missionGimbalPrepStage ==
        MISSION_GIMBAL_PREP_DONE);
}
