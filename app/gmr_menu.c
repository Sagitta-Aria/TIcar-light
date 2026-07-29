#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "menu.h"

#include "board.h"
#include "board_config.h"
#include "bluetooth_config.h"
#include "car_display.h"
#include "control_config.h"
#include "encoder_motor.h"
#include "gmr_bluetooth_mission.h"
#include "motor_no_yaw.h"
#include "task_registry.h"
#include "tuning_console.h"

#define GMR_MENU_LINE_SIZE          (24U)
#define GMR_MENU_MONO_CHARS         (18U)
#define GMR_MENU_PID_REFRESH_TICKS  (5U)
#define GMR_MENU_ENCODER_REFRESH_TICKS (1U)

typedef enum {
    GMR_MENU_PAGE_MAIN = 0,
    GMR_MENU_PAGE_DRIVE_MODE,
    GMR_MENU_PAGE_DRIVE_SPEED
} GmrMenuPage;

static GmrMenuPage g_page;
static uint8_t g_taskIndex;
static CarChassisDriveMode g_driveMode;
static uint16_t g_closedSpeed;
static uint16_t g_openSpeed;
static uint8_t g_forceRefresh;
static uint16_t g_refreshTicks;
static CarState g_lastState;

static char *GmrMenu_AppendText(char *write, char *end, const char *text)
{
    while ((text != 0) && (*text != '\0') && (write < end)) {
        *write++ = *text++;
    }
    *write = '\0';
    return write;
}

static char *GmrMenu_AppendChar(char *write, char *end, char value)
{
    if (write < end) {
        *write++ = value;
    }
    *write = '\0';
    return write;
}

static char *GmrMenu_AppendUnsigned(char *write, char *end, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < (uint8_t)sizeof(digits)));
    while ((count > 0U) && (write < end)) {
        *write++ = digits[--count];
    }
    *write = '\0';
    return write;
}

static char *GmrMenu_AppendSigned(char *write, char *end, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        write = GmrMenu_AppendChar(write, end, '-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    return GmrMenu_AppendUnsigned(write, end, magnitude);
}

static void GmrMenu_BuildPair(char line[GMR_MENU_LINE_SIZE],
    const char *label, int32_t left, int32_t right, const char *suffix)
{
    char *write = line;
    char *end = &line[GMR_MENU_LINE_SIZE - 1U];

    write = GmrMenu_AppendText(write, end, label);
    write = GmrMenu_AppendSigned(write, end, left);
    write = GmrMenu_AppendChar(write, end, ',');
    write = GmrMenu_AppendSigned(write, end, right);
    (void)GmrMenu_AppendText(write, end, suffix);
}

static void GmrMenu_BuildSignedLine(char line[GMR_MENU_LINE_SIZE],
    const char *label, int32_t value, const char *suffix)
{
    char *write = line;
    char *end = &line[GMR_MENU_LINE_SIZE - 1U];

    write = GmrMenu_AppendText(write, end, label);
    write = GmrMenu_AppendSigned(write, end, value);
    (void)GmrMenu_AppendText(write, end, suffix);
}

static void GmrMenu_ShowLine(uint8_t index, const char *text)
{
    char padded[GMR_MENU_MONO_CHARS + 1U];
    uint8_t i;

    for (i = 0U; i < GMR_MENU_MONO_CHARS; ++i) {
        padded[i] = ' ';
    }
    padded[GMR_MENU_MONO_CHARS] = '\0';
    for (i = 0U; (text != 0) && (text[i] != '\0') &&
        (i < GMR_MENU_MONO_CHARS); ++i) {
        padded[i] = text[i];
    }
    CarDisplay_ShowLine(index, padded);
}

static void GmrMenu_RenderLines(const char *line0, const char *line1,
    const char *line2, const char *line3)
{
    if (Board_IsDisplayAvailable() == 0U) {
        return;
    }
    GmrMenu_ShowLine(0U, line0);
    GmrMenu_ShowLine(1U, line1);
    GmrMenu_ShowLine(2U, line2);
    GmrMenu_ShowLine(3U, line3);
    /* H7有10行，非IMU页清空扩展遥测，避免切页后残留旧姿态。 */
    GmrMenu_ShowLine(4U, "");
    GmrMenu_ShowLine(5U, "");
    GmrMenu_ShowLine(6U, "");
    GmrMenu_ShowLine(7U, "");
    CarDisplay_Refresh();
}

/* yaw页在H7扩展行显示双IMU；本地OLED会按自身行数忽略超出部分。 */
static void GmrMenu_RenderImuTelemetry(const TuningConsoleDisplayStatus *status)
{
    char line4[GMR_MENU_LINE_SIZE];
    char line5[GMR_MENU_LINE_SIZE];
    char line6[GMR_MENU_LINE_SIZE];
    char line7[GMR_MENU_LINE_SIZE];

    GmrMenu_BuildPair(line4, "M0 Y/R ", status->gimbalYawX100,
        status->gimbalRateX100PerSec, "");
    GmrMenu_BuildPair(line5, "H7 R/P ", status->h7RollX100,
        status->h7PitchX100, "");
    GmrMenu_BuildPair(line6, "H7 Y/R ", status->h7YawX100,
        status->h7YawRateX100PerSec, "");
    GmrMenu_BuildPair(line7, "H7 F/A ", status->h7ImuFresh,
        (status->h7ImuAgeMs == 0xFFFFFFFFUL) ? -1 :
            (int32_t)status->h7ImuAgeMs, "");
    GmrMenu_ShowLine(4U, line4);
    GmrMenu_ShowLine(5U, line5);
    GmrMenu_ShowLine(6U, line6);
    GmrMenu_ShowLine(7U, line7);
    CarDisplay_Refresh();
}

static uint16_t GmrMenu_GetDriveSpeed(void)
{
    return (g_driveMode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ?
        g_closedSpeed : g_openSpeed;
}

static uint16_t GmrMenu_NextSpeed(uint16_t speed)
{
    speed = (uint16_t)(speed + (uint16_t)CHASSIS_DEBUG_SPEED_STEP);
    return (speed > (uint16_t)CHASSIS_DEBUG_SPEED_MAX) ?
        (uint16_t)CHASSIS_DEBUG_SPEED_MIN : speed;
}

static uint32_t GmrMenu_AbsEncoderCount(int32_t count)
{
    return (count < 0) ? (uint32_t)(-(count + 1)) + 1U :
        (uint32_t)count;
}

static void GmrMenu_BuildTaskLine(char line[GMR_MENU_LINE_SIZE],
    uint8_t taskIndex)
{
    char *write = line;
    char *end = &line[GMR_MENU_LINE_SIZE - 1U];

    write = GmrMenu_AppendChar(write, end,
        (taskIndex == g_taskIndex) ? '>' : ' ');
    (void)GmrMenu_AppendText(write, end,
        TaskRegistry_GetName(taskIndex));
}

static void GmrMenu_RenderMain(void)
{
    char line1[GMR_MENU_LINE_SIZE];
    char line2[GMR_MENU_LINE_SIZE];
    char line3[GMR_MENU_LINE_SIZE];
    uint8_t first = (g_taskIndex >= 2U) ?
        (uint8_t)(g_taskIndex - 1U) : 0U;

    if ((uint8_t)(first + 3U) > TaskRegistry_GetCount()) {
        first = (uint8_t)(TaskRegistry_GetCount() - 3U);
    }

    GmrMenu_BuildTaskLine(line1, first);
    GmrMenu_BuildTaskLine(line2, (uint8_t)(first + 1U));
    GmrMenu_BuildTaskLine(line3, (uint8_t)(first + 2U));
    GmrMenu_RenderLines("Menu 6", line1, line2, line3);
}

static void GmrMenu_RenderDriveMode(void)
{
    GmrMenu_RenderLines("Task 1 Mode",
        (g_driveMode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ?
            "[Closed] Open" : "Closed [Open]",
        "K1 Select", "K2 Speed");
}

static void GmrMenu_RenderDriveSpeed(void)
{
    char line0[GMR_MENU_LINE_SIZE];
    char line1[GMR_MENU_LINE_SIZE];
    char *write = line0;
    char *end = &line0[GMR_MENU_LINE_SIZE - 1U];

    write = GmrMenu_AppendText(write, end, "Task 1 ");
    (void)GmrMenu_AppendText(write, end,
        (g_driveMode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ? "Closed" : "Open");
    write = line1;
    end = &line1[GMR_MENU_LINE_SIZE - 1U];
    write = GmrMenu_AppendText(write, end, "Speed [");
    write = GmrMenu_AppendUnsigned(write, end, GmrMenu_GetDriveSpeed());
    (void)GmrMenu_AppendChar(write, end, ']');
    GmrMenu_RenderLines(line0, line1,
        (g_driveMode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ?
            "count/20ms" : "PWM percent", "K2 Start");
}

static int32_t GmrMenu_PwmPercent(int16_t pwm)
{
    return ((int32_t)pwm * 100L) / (int32_t)CHASSIS_PWM_LIMIT_COUNTS;
}

static void GmrMenu_RenderDriveMission(void)
{
    char line0[GMR_MENU_LINE_SIZE];
    char line1[GMR_MENU_LINE_SIZE];
    char line2[GMR_MENU_LINE_SIZE];
    char line3[GMR_MENU_LINE_SIZE];
    char *write = line0;
    char *end = &line0[GMR_MENU_LINE_SIZE - 1U];
    EncoderMotorSnapshot snapshot;

    EncoderMotor_GetSnapshot(&snapshot);
    write = GmrMenu_AppendText(write, end, "Task 1 ");
    (void)GmrMenu_AppendText(write, end,
        (StateMachine_GetMissionDriveMode(1U) ==
            CAR_CHASSIS_DRIVE_CLOSED_LOOP) ? "Closed" : "Open");
    GmrMenu_BuildPair(line1, "Set ",
        StateMachine_GetMissionDriveSpeed(1U),
        StateMachine_GetMissionDriveSpeed(1U), "");
    GmrMenu_BuildPair(line2, "F ",
        snapshot.feedbackCounts[ENCODER_MOTOR_LEFT],
        snapshot.feedbackCounts[ENCODER_MOTOR_RIGHT], " cnt");
    GmrMenu_BuildPair(line3, "PWM ",
        GmrMenu_PwmPercent(snapshot.outputPwm[ENCODER_MOTOR_LEFT]),
        GmrMenu_PwmPercent(snapshot.outputPwm[ENCODER_MOTOR_RIGHT]), "%");
    GmrMenu_RenderLines(line0, line1, line2, line3);
}

static void GmrMenu_RenderPidMission(void)
{
    char line1[GMR_MENU_LINE_SIZE];
    char line2[GMR_MENU_LINE_SIZE];
    char line3[GMR_MENU_LINE_SIZE];
    TuningConsoleDisplayStatus status;

    TuningConsole_GetDisplayStatus(&status);
    if (status.oledPage == TUNING_CONSOLE_OLED_GIMBAL) {
        GmrMenu_BuildPair(line1, "Y/T ", status.gimbalYawX100,
            status.gimbalYawX100 - status.gimbalAngleErrorX100, "");
        GmrMenu_BuildPair(line2, "E/C ", status.gimbalAngleErrorX100,
            status.gimbalCommandSps, "");
        GmrMenu_BuildPair(line3, "Fresh/Hold ",
            status.gimbalFeedbackFresh, status.gimbalHoldEnabled, "");
        GmrMenu_RenderLines((StateMachine_GetMissionId() == 5U) ?
            "Task 5 M0 Yaw" : "Task 2 M0 Yaw", line1, line2, line3);
        GmrMenu_RenderImuTelemetry(&status);
    } else if (status.oledPage == TUNING_CONSOLE_OLED_START) {
        GmrMenu_BuildPair(line1, "Cnt ", status.leftFeedbackCounts,
            status.rightFeedbackCounts, "");
        GmrMenu_BuildPair(line2, "Start ", status.leftStartPercent,
            status.rightStartPercent, "%");
        GmrMenu_BuildPair(line3, "Run ", status.leftRunStartPercent,
            status.rightRunStartPercent, "%");
        GmrMenu_RenderLines((StateMachine_GetMissionId() == 5U) ?
            "Task 5 START" : "Task 2 START", line1, line2, line3);
    } else if (status.oledPage == TUNING_CONSOLE_OLED_SPEED) {
        GmrMenu_BuildPair(line1, "PWM ", status.leftPwmPercent,
            status.rightPwmPercent, "%");
        GmrMenu_BuildPair(line2, "Avg ", status.leftAverageCounts,
            status.rightAverageCounts, "");
        GmrMenu_BuildPair(line3, "CPS ",
            status.leftAverageCounts * (int32_t)CHASSIS_SPEED_UNIT_HZ,
            status.rightAverageCounts * (int32_t)CHASSIS_SPEED_UNIT_HZ, "");
        GmrMenu_RenderLines((StateMachine_GetMissionId() == 5U) ?
            "Task 5 SPEED" : "Task 2 SPEED", line1, line2, line3);
    } else if (status.oledPage == TUNING_CONSOLE_OLED_PID) {
        GmrMenu_BuildPair(line1, "T ", status.leftTargetCounts,
            status.rightTargetCounts, "");
        GmrMenu_BuildPair(line2, "F ", status.leftFeedbackCounts,
            status.rightFeedbackCounts, "");
        GmrMenu_BuildPair(line3, "PWM ", status.leftPwmPercent,
            status.rightPwmPercent, "%");
        GmrMenu_RenderLines((StateMachine_GetMissionId() == 5U) ?
            "Task 5 PID" : "Task 2 PID", line1, line2, line3);
    } else {
        GmrMenu_BuildPair(line1, "FF ", status.leftFfQ1024,
            status.rightFfQ1024, "");
        GmrMenu_BuildPair(line2, "Start ", status.leftStartPercent,
            status.rightStartPercent, "%");
        GmrMenu_BuildPair(line3, "Run ", status.leftRunStartPercent,
            status.rightRunStartPercent, "%");
        GmrMenu_RenderLines((StateMachine_GetMissionId() == 5U) ?
            "Task 5 FF" : "Task 2 FF", line1, line2, line3);
    }
}

static void GmrMenu_RenderEncoderMission(void)
{
    char leftLine[GMR_MENU_LINE_SIZE];
    char rightLine[GMR_MENU_LINE_SIZE];
    char averageLine[GMR_MENU_LINE_SIZE];
    int32_t leftCount = EncoderMotor_GetTotalCount(ENCODER_MOTOR_LEFT);
    int32_t rightCount = EncoderMotor_GetTotalCount(ENCODER_MOTOR_RIGHT);
    uint32_t averageCount = (GmrMenu_AbsEncoderCount(leftCount) +
        GmrMenu_AbsEncoderCount(rightCount)) / 2U;

    GmrMenu_BuildSignedLine(leftLine, "L ", leftCount, "");
    GmrMenu_BuildSignedLine(rightLine, "R ", rightCount, "");
    GmrMenu_BuildSignedLine(averageLine, "Avg ", (int32_t)averageCount,
        "");
    GmrMenu_RenderLines("Task 3 Encoder", leftLine, rightLine,
        averageLine);
}

static void GmrMenu_RenderLineMission(void)
{
    char line1[GMR_MENU_LINE_SIZE];
    char line2[GMR_MENU_LINE_SIZE];
    char line3[GMR_MENU_LINE_SIZE];
    EncoderMotorSnapshot motor;
    char *write = line1;
    char *end = &line1[GMR_MENU_LINE_SIZE - 1U];

    EncoderMotor_GetSnapshot(&motor);
    write = GmrMenu_AppendText(write, end, "St ");
    write = GmrMenu_AppendText(write, end, MotorNoYaw_GetStateName());
    write = GmrMenu_AppendText(write, end, " T");
    (void)GmrMenu_AppendUnsigned(write, end, MotorNoYaw_GetTurnCount());
    GmrMenu_BuildPair(line2, "M/E ", MotorNoYaw_GetDigitalMask(),
        MotorNoYaw_GetLineError(), "");
    GmrMenu_BuildPair(line3, "F ",
        motor.feedbackCounts[ENCODER_MOTOR_LEFT],
        motor.feedbackCounts[ENCODER_MOTOR_RIGHT], "");
    GmrMenu_RenderLines("Task 4 Line", line1, line2, line3);
}

static void GmrMenu_RenderBluetoothMission(void)
{
    char line0[GMR_MENU_LINE_SIZE];
    char line1[GMR_MENU_LINE_SIZE];
    char line2[GMR_MENU_LINE_SIZE];
    char line3[GMR_MENU_LINE_SIZE];
    char *write;
    char *end;
    GmrBluetoothMissionStatus status;

    GmrBluetoothMission_GetStatus(&status);
    write = line0;
    end = &line0[GMR_MENU_LINE_SIZE - 1U];
    write = GmrMenu_AppendText(write, end, "Task 6 ");
    (void)GmrMenu_AppendText(write, end,
        (status.role == CAR_BLUETOOTH_ROLE_MASTER) ? "Master" :
        ((status.role == CAR_BLUETOOTH_ROLE_SLAVE) ? "Slave" : "Off"));

    write = line1;
    end = &line1[GMR_MENU_LINE_SIZE - 1U];
    write = GmrMenu_AppendText(write, end, "Link ");
    write = GmrMenu_AppendUnsigned(write, end, status.connected);
    write = GmrMenu_AppendChar(write, end, ' ');
    (void)GmrMenu_AppendText(write, end,
        GmrBluetoothMission_GetPhaseName(status.phase));

    write = line2;
    end = &line2[GMR_MENU_LINE_SIZE - 1U];
    if (status.role == CAR_BLUETOOTH_ROLE_MASTER) {
        write = GmrMenu_AppendText(write, end, "Tx ");
        write = GmrMenu_AppendUnsigned(write, end, status.sampleCount);
        write = GmrMenu_AppendText(write, end, " T");
        write = GmrMenu_AppendUnsigned(write, end, status.completedTurns);
        write = GmrMenu_AppendChar(write, end, '/');
        (void)GmrMenu_AppendUnsigned(write, end,
            GMR_BLUETOOTH_MISSION_TARGET_TURNS);
    } else if (status.phase == GMR_BLUETOOTH_MISSION_PHASE_REPLAY) {
        write = GmrMenu_AppendText(write, end, "Play ");
        write = GmrMenu_AppendUnsigned(write, end, status.replayIndex);
        write = GmrMenu_AppendChar(write, end, '/');
        (void)GmrMenu_AppendUnsigned(write, end, status.sampleCount);
    } else {
        write = GmrMenu_AppendText(write, end, "Rx ");
        write = GmrMenu_AppendUnsigned(write, end, status.sampleCount);
        write = GmrMenu_AppendChar(write, end, '/');
        (void)GmrMenu_AppendUnsigned(write, end,
            GMR_BLUETOOTH_MISSION_MAX_SAMPLES);
    }

    write = line3;
    end = &line3[GMR_MENU_LINE_SIZE - 1U];
    if (status.error != GMR_BLUETOOTH_MISSION_ERROR_NONE) {
        write = GmrMenu_AppendText(write, end, "Err ");
        (void)GmrMenu_AppendText(write, end,
            GmrBluetoothMission_GetErrorName(status.error));
    } else if ((status.role == CAR_BLUETOOTH_ROLE_SLAVE) &&
        (status.phase == GMR_BLUETOOTH_MISSION_PHASE_REPLAY)) {
        GmrMenu_BuildPair(line3, "PosE ",
            status.replayLeftErrorCounts,
            status.replayRightErrorCounts, "");
    } else if (status.role == CAR_BLUETOOTH_ROLE_SLAVE) {
        write = GmrMenu_AppendText(write, end, "Play ");
        write = GmrMenu_AppendUnsigned(write, end, status.replayIndex);
        write = GmrMenu_AppendChar(write, end, '/');
        (void)GmrMenu_AppendUnsigned(write, end, status.sampleCount);
    } else {
        write = GmrMenu_AppendText(write, end, "Cap ");
        (void)GmrMenu_AppendUnsigned(write, end,
            GMR_BLUETOOTH_MISSION_MAX_SAMPLES);
    }
    GmrMenu_RenderLines(line0, line1, line2, line3);
}

static void GmrMenu_Render(CarState state)
{
    if (state == CAR_STATE_MENU) {
        if (g_page == GMR_MENU_PAGE_DRIVE_MODE) {
            GmrMenu_RenderDriveMode();
        } else if (g_page == GMR_MENU_PAGE_DRIVE_SPEED) {
            GmrMenu_RenderDriveSpeed();
        } else {
            GmrMenu_RenderMain();
        }
    } else if (state == CAR_STATE_MISSION) {
        if (StateMachine_GetMissionId() == 3U) {
            GmrMenu_RenderEncoderMission();
        } else if (StateMachine_GetMissionId() == 4U) {
            GmrMenu_RenderLineMission();
        } else if (StateMachine_GetMissionId() == 1U) {
            GmrMenu_RenderDriveMission();
        } else if (StateMachine_GetMissionId() == 6U) {
            GmrMenu_RenderBluetoothMission();
        } else {
            GmrMenu_RenderPidMission();
        }
    } else if (state == CAR_STATE_STOP) {
        GmrMenu_RenderLines("Stop", "K2 Back", "", "");
    } else if (state == CAR_STATE_ERROR) {
        if (StateMachine_GetMissionId() == 6U) {
            GmrMenu_RenderBluetoothMission();
        } else {
            GmrMenu_RenderLines("Error", "K2 Back", "", "");
        }
    } else if (state == CAR_STATE_FINISHED) {
        if (StateMachine_GetMissionId() == 6U) {
            GmrMenu_RenderBluetoothMission();
        } else {
            GmrMenu_RenderLines("Finished", "K2 Back", "", "");
        }
    } else {
        GmrMenu_RenderLines("Init", "", "", "");
    }
}

void Menu_Init(void)
{
    g_page = GMR_MENU_PAGE_MAIN;
    g_taskIndex = 0U;
    g_driveMode = (CHASSIS_TASK1_DEFAULT_CLOSED_LOOP != 0U) ?
        CAR_CHASSIS_DRIVE_CLOSED_LOOP : CAR_CHASSIS_DRIVE_OPEN_LOOP;
    g_closedSpeed = (uint16_t)CHASSIS_TASK1_CLOSED_SPEED_DEFAULT;
    g_openSpeed = (uint16_t)CHASSIS_TASK1_OPEN_SPEED_DEFAULT;
    g_forceRefresh = 1U;
    g_refreshTicks = CAR_MENU_REFRESH_TICKS;
    g_lastState = CAR_STATE_INIT;
}

void Menu_Next(void)
{
    if (g_page == GMR_MENU_PAGE_DRIVE_MODE) {
        g_driveMode = (g_driveMode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ?
            CAR_CHASSIS_DRIVE_OPEN_LOOP : CAR_CHASSIS_DRIVE_CLOSED_LOOP;
    } else if (g_page == GMR_MENU_PAGE_DRIVE_SPEED) {
        if (g_driveMode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) {
            g_closedSpeed = GmrMenu_NextSpeed(g_closedSpeed);
        } else {
            g_openSpeed = GmrMenu_NextSpeed(g_openSpeed);
        }
    } else {
        ++g_taskIndex;
        if (g_taskIndex >= TaskRegistry_GetCount()) {
            g_taskIndex = 0U;
        }
    }
    Menu_RequestRefresh();
}

CarEvent Menu_Confirm(void)
{
    if (g_page == GMR_MENU_PAGE_DRIVE_MODE) {
        g_page = GMR_MENU_PAGE_DRIVE_SPEED;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    }
    if (g_page == GMR_MENU_PAGE_DRIVE_SPEED) {
        StateMachine_SetMissionDriveConfig(1U, g_driveMode,
            GmrMenu_GetDriveSpeed());
        return TaskRegistry_GetStartEvent(0U);
    }
    if (g_taskIndex == 0U) {
        g_page = GMR_MENU_PAGE_DRIVE_MODE;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    }
    return TaskRegistry_GetStartEvent(g_taskIndex);
}

uint8_t Menu_Back(void)
{
    if (g_page == GMR_MENU_PAGE_MAIN) {
        return 0U;
    }
    g_page = (g_page == GMR_MENU_PAGE_DRIVE_SPEED) ?
        GMR_MENU_PAGE_DRIVE_MODE : GMR_MENU_PAGE_MAIN;
    Menu_RequestRefresh();
    return 1U;
}

void Menu_RequestRefresh(void)
{
    g_forceRefresh = 1U;
}

void Menu_Task(CarState state)
{
    uint16_t refreshTicks = CAR_MENU_REFRESH_TICKS;

    if ((state == CAR_STATE_MISSION) &&
        (StateMachine_GetMissionId() == 3U)) {
        refreshTicks = GMR_MENU_ENCODER_REFRESH_TICKS;
    } else if ((state == CAR_STATE_MISSION) &&
        (StateMachine_GetMissionId() == 4U)) {
        refreshTicks = GMR_MENU_ENCODER_REFRESH_TICKS;
    } else if ((state == CAR_STATE_MISSION) &&
        ((StateMachine_GetMissionId() == 2U) ||
            (StateMachine_GetMissionId() == 5U))) {
        refreshTicks = GMR_MENU_PID_REFRESH_TICKS;
    }

    if (state != g_lastState) {
        if (state == CAR_STATE_MENU) {
            g_page = GMR_MENU_PAGE_MAIN;
        }
        g_lastState = state;
        g_forceRefresh = 1U;
        g_refreshTicks = refreshTicks;
    } else if (g_refreshTicks < refreshTicks) {
        ++g_refreshTicks;
    }
    if ((g_forceRefresh == 0U) && (g_refreshTicks < refreshTicks)) {
        return;
    }
    g_forceRefresh = 0U;
    g_refreshTicks = 0U;
    GmrMenu_Render(state);
    if (Board_IsDisplayAvailable() == 0U) {
        g_forceRefresh = 1U;
    }
}

#endif
