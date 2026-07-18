#include "menu.h"

#include "board.h"
#include "board_config.h"
#include "control_config.h"
#include "encoder_motor.h"
#include "gimbal_attitude.h"
#include "motor_no_yaw.h"
#include "oled.h"
#include "tuning_console.h"

#define MENU_TASK_COUNT       (8U)
#define MENU_GIMBAL_DISTANCE_COUNT (3U)
#define MENU_OLED_FONT_SIZE   (12U)
#define MENU_MONO_START_X     (6U)
#define MENU_MONO_START_Y     (16U)
#define MENU_MONO_LINE_STEP   (12U)
#define MENU_MONO_MAX_CHARS   (18U)
#define MENU_LINE_SIZE        (24U)
#define MENU_TASK5_REFRESH_TICKS (5U)

static const char *const g_taskNames[MENU_TASK_COUNT] = {
    "Task 1",
    "Task 2",
    "Task 3",
    "Task 4",
    "Task 5 PID",
    "Task 6 Drive",
    "Task 7 Circle",
    "Task 8 IMU"
};

static const char *const g_gimbalDistanceNames[MENU_GIMBAL_DISTANCE_COUNT] = {
    "Near",
    "Mid",
    "Far"
};

typedef enum {
    MENU_PAGE_MAIN = 0,
    MENU_PAGE_TASK1_LAPS,
    MENU_PAGE_TASK2_DISTANCE,
    MENU_PAGE_TASK3_DISTANCE,
    MENU_PAGE_TASK7_DISTANCE,
    MENU_PAGE_TASK6_MODE,
    MENU_PAGE_TASK6_SPEED
} MenuPage;

static MenuPage g_menuPage;
static uint8_t g_taskIndex;
static uint8_t g_task1LapCount;
static uint8_t g_task2Distance;
static uint8_t g_task3Distance;
static uint8_t g_task7Distance;
static CarChassisDriveMode g_task6DriveMode;
static uint16_t g_task6ClosedSpeed;
static uint16_t g_task6OpenSpeed;
static uint8_t g_forceRefresh;
static uint16_t g_refreshTicks;
static CarState g_lastState;

static char *Menu_AppendText(char *write, char *end, const char *text)
{
    while ((text != 0) && (*text != '\0') && (write < end)) {
        *write = *text;
        ++write;
        ++text;
    }
    *write = '\0';
    return write;
}

static char *Menu_AppendChar(char *write, char *end, char value)
{
    if (write < end) {
        *write = value;
        ++write;
    }
    *write = '\0';
    return write;
}

static char *Menu_AppendUnsigned(char *write, char *end, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count] = (char)('0' + (value % 10U));
        value /= 10U;
        ++count;
    } while ((value != 0U) && (count < (uint8_t)sizeof(digits)));

    while ((count > 0U) && (write < end)) {
        --count;
        *write = digits[count];
        ++write;
    }
    *write = '\0';
    return write;
}

static char *Menu_AppendSigned(char *write, char *end, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        write = Menu_AppendChar(write, end, '-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    return Menu_AppendUnsigned(write, end, magnitude);
}

static char *Menu_AppendFf(char *write, char *end, int32_t ffQ1024)
{
    uint32_t magnitude;
    uint32_t whole;
    uint32_t fraction;

    if (ffQ1024 < 0) {
        write = Menu_AppendChar(write, end, '-');
        magnitude = (uint32_t)(-(ffQ1024 + 1)) + 1U;
    } else {
        magnitude = (uint32_t)ffQ1024;
    }

    whole = magnitude / 1024U;
    fraction = (((magnitude % 1024U) * 1000U) + 512U) / 1024U;
    if (fraction >= 1000U) {
        ++whole;
        fraction = 0U;
    }
    write = Menu_AppendUnsigned(write, end, whole);
    write = Menu_AppendChar(write, end, '.');
    if (fraction < 100U) {
        write = Menu_AppendChar(write, end, '0');
    }
    if (fraction < 10U) {
        write = Menu_AppendChar(write, end, '0');
    }
    return Menu_AppendUnsigned(write, end, fraction);
}

static void Menu_BuildSignedPair(char line[MENU_LINE_SIZE],
    const char *label, int32_t left, int32_t right, const char *suffix)
{
    char *write = line;
    char *end = &line[MENU_LINE_SIZE - 1U];

    write = Menu_AppendText(write, end, label);
    write = Menu_AppendSigned(write, end, left);
    write = Menu_AppendChar(write, end, ',');
    write = Menu_AppendSigned(write, end, right);
    (void)Menu_AppendText(write, end, suffix);
}

static void Menu_BuildGrayMaskLine(char line[MENU_LINE_SIZE], uint8_t mask)
{
    char *write = line;
    char *end = &line[MENU_LINE_SIZE - 1U];
    uint8_t bit;

    write = Menu_AppendText(write, end, "S1-7 ");
    for (bit = 0U; bit < GRAY_SENSOR_COUNT; ++bit) {
        write = Menu_AppendChar(write, end,
            ((mask & (uint8_t)(0x40U >> bit)) != 0U) ? '1' : '0');
    }
}

static void Menu_ShowMonoLine(uint8_t index, const char *text)
{
    char padded[MENU_MONO_MAX_CHARS + 1U];
    uint8_t i;
    uint8_t y;

    for (i = 0U; i < MENU_MONO_MAX_CHARS; ++i) {
        padded[i] = ' ';
    }
    padded[MENU_MONO_MAX_CHARS] = '\0';

    i = 0U;
    while ((text != 0) && (text[i] != '\0') &&
        (i < MENU_MONO_MAX_CHARS)) {
        padded[i] = text[i];
        ++i;
    }

    y = (uint8_t)(MENU_MONO_START_Y + (index * MENU_MONO_LINE_STEP));
    OLED_ShowString(MENU_MONO_START_X, y, (u8 *)padded,
        MENU_OLED_FONT_SIZE);
}

static void Menu_RenderLines(const char *line0, const char *line1,
    const char *line2, const char *line3)
{
    if (Board_IsOledAvailable() == 0U) {
        return;
    }

    Menu_ShowMonoLine(0U, line0);
    Menu_ShowMonoLine(1U, line1);
    Menu_ShowMonoLine(2U, line2);
    Menu_ShowMonoLine(3U, line3);
    OLED_Refresh();
}

static void Menu_BuildTaskLine(char line[MENU_LINE_SIZE], uint8_t task,
    uint8_t selected)
{
    char *write = line;
    char *end = &line[MENU_LINE_SIZE - 1U];

    write = Menu_AppendChar(write, end, selected ? '>' : ' ');
    (void)Menu_AppendText(write, end, g_taskNames[task]);
}

static void Menu_RenderTaskMenu(void)
{
    char line0[MENU_LINE_SIZE];
    char line1[MENU_LINE_SIZE];
    char line2[MENU_LINE_SIZE];
    char line3[MENU_LINE_SIZE];
    uint8_t prev;
    uint8_t next;
    uint8_t next2;

    prev = (g_taskIndex == 0U) ? (MENU_TASK_COUNT - 1U) :
        (uint8_t)(g_taskIndex - 1U);
    next = (uint8_t)((g_taskIndex + 1U) % MENU_TASK_COUNT);
    next2 = (uint8_t)((g_taskIndex + 2U) % MENU_TASK_COUNT);

    Menu_BuildTaskLine(line0, prev, 0U);
    Menu_BuildTaskLine(line1, g_taskIndex, 1U);
    Menu_BuildTaskLine(line2, next, 0U);
    Menu_BuildTaskLine(line3, next2, 0U);
    Menu_RenderLines(line0, line1, line2, line3);
}

static void Menu_RenderTask1LapMenu(void)
{
    char optionLine[MENU_LINE_SIZE];
    char lapLine[MENU_LINE_SIZE];
    char *write;
    char *end;
    uint8_t i;

    write = optionLine;
    end = &optionLine[MENU_LINE_SIZE - 1U];
    for (i = 1U; i <= 5U; ++i) {
        if (i == g_task1LapCount) {
            write = Menu_AppendChar(write, end, '[');
            write = Menu_AppendUnsigned(write, end, i);
            write = Menu_AppendChar(write, end, ']');
        } else {
            write = Menu_AppendUnsigned(write, end, i);
        }
        if (i < 5U) {
            write = Menu_AppendChar(write, end, ' ');
        }
    }

    write = lapLine;
    end = &lapLine[MENU_LINE_SIZE - 1U];
    write = Menu_AppendText(write, end, "Lap ");
    (void)Menu_AppendUnsigned(write, end, g_task1LapCount);

    Menu_RenderLines("Task 1", optionLine, lapLine, "K2 Start");
}

static void Menu_RenderGimbalDistanceMenu(const char *title,
    uint8_t distance)
{
    char optionLine[MENU_LINE_SIZE];
    char distanceLine[MENU_LINE_SIZE];
    char *write;
    char *end;
    uint8_t i;

    write = optionLine;
    end = &optionLine[MENU_LINE_SIZE - 1U];
    for (i = 0U; i < MENU_GIMBAL_DISTANCE_COUNT; ++i) {
        if (i == distance) {
            write = Menu_AppendChar(write, end, '[');
            write = Menu_AppendText(write, end, g_gimbalDistanceNames[i]);
            write = Menu_AppendChar(write, end, ']');
        } else {
            write = Menu_AppendText(write, end, g_gimbalDistanceNames[i]);
        }
        if (i < (MENU_GIMBAL_DISTANCE_COUNT - 1U)) {
            write = Menu_AppendChar(write, end, ' ');
        }
    }

    write = distanceLine;
    end = &distanceLine[MENU_LINE_SIZE - 1U];
    write = Menu_AppendText(write, end, "Distance ");
    (void)Menu_AppendText(write, end, g_gimbalDistanceNames[distance]);

    Menu_RenderLines(title, optionLine, distanceLine, "K2 Start");
}

static CarChassisDriveMode Menu_GetDriveMode(uint8_t missionId)
{
    (void)missionId;
    return g_task6DriveMode;
}

static uint16_t Menu_GetDriveSpeed(uint8_t missionId)
{
    CarChassisDriveMode mode = Menu_GetDriveMode(missionId);

    return (mode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ?
        g_task6ClosedSpeed : g_task6OpenSpeed;
}

static uint16_t Menu_NextDriveSpeed(uint16_t speed)
{
    speed = (uint16_t)(speed + (uint16_t)CHASSIS_DEBUG_SPEED_STEP);
    return (speed > (uint16_t)CHASSIS_DEBUG_SPEED_MAX) ?
        (uint16_t)CHASSIS_DEBUG_SPEED_MIN : speed;
}

static void Menu_RenderDriveModeMenu(uint8_t missionId)
{
    char title[MENU_LINE_SIZE];
    char optionLine[MENU_LINE_SIZE];
    char *write = title;
    char *end = &title[MENU_LINE_SIZE - 1U];
    CarChassisDriveMode mode = Menu_GetDriveMode(missionId);

    write = Menu_AppendText(write, end, "Task ");
    write = Menu_AppendUnsigned(write, end, missionId);
    (void)Menu_AppendText(write, end, " Mode");
    if (mode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) {
        (void)Menu_AppendText(optionLine,
            &optionLine[MENU_LINE_SIZE - 1U], "[Closed] Open");
    } else {
        (void)Menu_AppendText(optionLine,
            &optionLine[MENU_LINE_SIZE - 1U], "Closed [Open]");
    }
    Menu_RenderLines(title, optionLine, "K1 Select", "K2 Speed");
}

static void Menu_RenderDriveSpeedMenu(uint8_t missionId)
{
    char title[MENU_LINE_SIZE];
    char speedLine[MENU_LINE_SIZE];
    char *write = title;
    char *end = &title[MENU_LINE_SIZE - 1U];
    CarChassisDriveMode mode = Menu_GetDriveMode(missionId);

    write = Menu_AppendText(write, end, "Task ");
    write = Menu_AppendUnsigned(write, end, missionId);
    write = Menu_AppendChar(write, end, ' ');
    (void)Menu_AppendText(write, end,
        (mode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ? "Closed" : "Open");

    write = speedLine;
    end = &speedLine[MENU_LINE_SIZE - 1U];
    write = Menu_AppendText(write, end, "Speed [");
    write = Menu_AppendUnsigned(write, end, Menu_GetDriveSpeed(missionId));
    (void)Menu_AppendChar(write, end, ']');
    Menu_RenderLines(title, speedLine,
        (mode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ?
            "count/20ms" : "PWM percent", "K2 Start");
}

static void Menu_RenderTask5(void)
{
    char line0[MENU_LINE_SIZE];
    char line1[MENU_LINE_SIZE];
    char line2[MENU_LINE_SIZE];
    char line3[MENU_LINE_SIZE];
    char *write;
    char *end;
    TuningConsoleDisplayStatus status;

    TuningConsole_GetDisplayStatus(&status);
    if (status.visionMode != 0U) {
        write = line1;
        end = &line1[MENU_LINE_SIZE - 1U];
        if (status.visionHasFrame != 0U) {
            write = Menu_AppendText(write, end, "FRAME ");
            (void)Menu_AppendUnsigned(write, end,
                status.visionFrameCount);
        } else {
            (void)Menu_AppendText(write, end, "WAIT FRAME");
        }
        Menu_BuildSignedPair(line2, "E ", status.visionRawX,
            status.visionRawY, "");
        Menu_BuildSignedPair(line3, "S ", status.visionCommandX,
            status.visionCommandY, "");
        Menu_RenderLines("Task 5 VISION", line1, line2, line3);
        return;
    }
    if (status.gimbalMode != 0U) {
        write = line1;
        end = &line1[MENU_LINE_SIZE - 1U];
        if (status.gimbalState == (uint8_t)BODY_MOTION_CALIBRATING) {
            write = Menu_AppendText(write, end, "CAL ");
            write = Menu_AppendUnsigned(write, end,
                status.gimbalCalibrationCount);
            write = Menu_AppendChar(write, end, '/');
            (void)Menu_AppendUnsigned(write, end,
                status.gimbalCalibrationTarget);
        } else if (status.gimbalState == (uint8_t)BODY_MOTION_READY) {
            write = Menu_AppendText(write, end,
                (status.gimbalHoldEnabled != 0U) ? "HOLD" : "OBSERVE");
            write = Menu_AppendText(write, end, " FF");
            (void)Menu_AppendUnsigned(write, end,
                status.gimbalFeedForwardEnabled);
        } else if (status.gimbalState == (uint8_t)BODY_MOTION_STALE) {
            (void)Menu_AppendText(write, end, "IMU STALE");
        } else {
            (void)Menu_AppendText(write, end, "WAIT IMU");
        }
        write = line2;
        end = &line2[MENU_LINE_SIZE - 1U];
        write = Menu_AppendText(write, end, "Y100 ");
        write = Menu_AppendSigned(write, end, status.gimbalYawX100);
        write = Menu_AppendText(write, end, " R ");
        (void)Menu_AppendSigned(write, end,
            status.gimbalRateX100PerSec);
        write = line3;
        end = &line3[MENU_LINE_SIZE - 1U];
        write = Menu_AppendText(write, end, "E ");
        write = Menu_AppendSigned(write, end, status.gimbalStepError);
        write = Menu_AppendText(write, end, " S ");
        (void)Menu_AppendSigned(write, end, status.gimbalCommandSps);
        Menu_RenderLines("Task 5 GIMBAL", line1, line2, line3);
        return;
    }
    if (status.oledPage == TUNING_CONSOLE_OLED_GRAY) {
        Menu_BuildGrayMaskLine(line1, status.grayMask);
        Menu_BuildSignedPair(line2, "T ", status.leftTargetCounts,
            status.rightTargetCounts, "");
        Menu_BuildSignedPair(line3, "F ", status.leftFeedbackCounts,
            status.rightFeedbackCounts, "");
        Menu_RenderLines("Task 5 GRAY", line1, line2, line3);
        return;
    }
    if (status.oledPage == TUNING_CONSOLE_OLED_START) {
        Menu_BuildSignedPair(line1, "Cnt ", status.leftFeedbackCounts,
            status.rightFeedbackCounts, "");
        Menu_BuildSignedPair(line2, "Start ", status.leftStartPercent,
            status.rightStartPercent, "%");
        Menu_BuildSignedPair(line3, "Run ", status.leftRunStartPercent,
            status.rightRunStartPercent, "%");
        Menu_RenderLines("Task 5 START", line1, line2, line3);
        return;
    }
    if (status.oledPage == TUNING_CONSOLE_OLED_SPEED) {
        Menu_BuildSignedPair(line1, "PWM ", status.leftPwmPercent,
            status.rightPwmPercent, "%");
        Menu_BuildSignedPair(line2, "Avg ", status.leftAverageCounts,
            status.rightAverageCounts, "");
        Menu_BuildSignedPair(line3, "CPS ",
            status.leftAverageCounts *
                (1000L / (int32_t)CHASSIS_CONTROL_PERIOD_MS),
            status.rightAverageCounts *
                (1000L / (int32_t)CHASSIS_CONTROL_PERIOD_MS), "");
        Menu_RenderLines("Task 5 SPEED", line1, line2, line3);
        return;
    }
    if (status.oledPage == TUNING_CONSOLE_OLED_PID) {
        Menu_BuildSignedPair(line1, "T ", status.leftTargetCounts,
            status.rightTargetCounts, "");
        Menu_BuildSignedPair(line2, "F ", status.leftFeedbackCounts,
            status.rightFeedbackCounts, "");
        Menu_BuildSignedPair(line3, "PWM ", status.leftPwmPercent,
            status.rightPwmPercent, "%");
        Menu_RenderLines("Task 5 PID", line1, line2, line3);
        return;
    }

    write = line1;
    end = &line1[MENU_LINE_SIZE - 1U];
    write = Menu_AppendText(write, end, "L FF ");
    if (status.leftResult == TUNING_CONSOLE_SET_RESULT_FF_APPLIED) {
        (void)Menu_AppendFf(write, end, status.leftFfQ1024);
    } else {
        (void)Menu_AppendText(write, end, "--");
    }

    write = line2;
    end = &line2[MENU_LINE_SIZE - 1U];
    write = Menu_AppendText(write, end, "R FF ");
    if (status.rightResult == TUNING_CONSOLE_SET_RESULT_FF_APPLIED) {
        (void)Menu_AppendFf(write, end, status.rightFfQ1024);
    } else {
        (void)Menu_AppendText(write, end, "--");
    }
    Menu_BuildSignedPair(line3, "Run ", status.leftRunStartPercent,
        status.rightRunStartPercent, "%");

    write = line0;
    end = &line0[MENU_LINE_SIZE - 1U];
    (void)Menu_AppendText(write, end, "Task 5 FF");
    Menu_RenderLines(line0, line1, line2, line3);
}

static int32_t Menu_PwmCountsToPercent(int16_t pwm)
{
    return ((int32_t)pwm * 100L) / (int32_t)CHASSIS_PWM_LIMIT_COUNTS;
}

static void Menu_BuildDriveTitle(char line[MENU_LINE_SIZE],
    uint8_t missionId, CarChassisDriveMode mode)
{
    char *write = line;
    char *end = &line[MENU_LINE_SIZE - 1U];

    write = Menu_AppendText(write, end, "Task ");
    write = Menu_AppendUnsigned(write, end, missionId);
    write = Menu_AppendChar(write, end, ' ');
    (void)Menu_AppendText(write, end,
        (mode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ? "Closed" : "Open");
}

static void Menu_BuildDriveSetLine(char line[MENU_LINE_SIZE],
    CarChassisDriveMode mode, uint16_t speed, uint8_t rightOnly)
{
    char *write = line;
    char *end = &line[MENU_LINE_SIZE - 1U];

    if (rightOnly != 0U) {
        write = Menu_AppendText(write, end, "R ");
    }
    write = Menu_AppendText(write, end,
        (mode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ? "Set " : "PWM ");
    write = Menu_AppendUnsigned(write, end, speed);
    (void)Menu_AppendText(write, end,
        (mode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) ? " cnt/20" : "%");
}

static void Menu_RenderTask6Mission(void)
{
    char line0[MENU_LINE_SIZE];
    char line1[MENU_LINE_SIZE];
    char line2[MENU_LINE_SIZE];
    char line3[MENU_LINE_SIZE];
    EncoderMotorSnapshot snapshot;
    CarChassisDriveMode mode = StateMachine_GetMissionDriveMode(6U);

    EncoderMotor_GetSnapshot(&snapshot);
    Menu_BuildDriveTitle(line0, 6U, mode);
    Menu_BuildDriveSetLine(line1, mode,
        StateMachine_GetMissionDriveSpeed(6U), 0U);
    Menu_BuildSignedPair(line2, "F ",
        snapshot.feedbackCounts[ENCODER_MOTOR_LEFT],
        snapshot.feedbackCounts[ENCODER_MOTOR_RIGHT], " cnt");
    Menu_BuildSignedPair(line3, "PWM ",
        Menu_PwmCountsToPercent(snapshot.outputPwm[ENCODER_MOTOR_LEFT]),
        Menu_PwmCountsToPercent(snapshot.outputPwm[ENCODER_MOTOR_RIGHT]), "%");
    Menu_RenderLines(line0, line1, line2, line3);
}

static void Menu_RenderMission(void)
{
    char line[MENU_LINE_SIZE];
    char lapLine[MENU_LINE_SIZE];
    char turnLine[MENU_LINE_SIZE];
    char *write = line;
    char *end = &line[MENU_LINE_SIZE - 1U];
    uint8_t missionId = StateMachine_GetMissionId();
    uint8_t targetLaps;
    uint32_t turnCount;
    uint32_t targetTurns;

    write = Menu_AppendText(write, end, "Task ");
    (void)Menu_AppendUnsigned(write, end, missionId);

    if (missionId == 5U) {
        Menu_RenderTask5();
        return;
    }

    if (missionId == 6U) {
        Menu_RenderTask6Mission();
        return;
    }

    if (missionId == 8U) {
        GimbalAttitudeSnapshot status;
        char stateLine[MENU_LINE_SIZE];
        char yawLine[MENU_LINE_SIZE];
        char controlLine[MENU_LINE_SIZE];
        char *stateWrite = stateLine;
        char *yawWrite = yawLine;
        char *controlWrite = controlLine;
        char *stateEnd = &stateLine[MENU_LINE_SIZE - 1U];
        char *yawEnd = &yawLine[MENU_LINE_SIZE - 1U];
        char *controlEnd = &controlLine[MENU_LINE_SIZE - 1U];

        GimbalAttitude_GetSnapshot(&status);
        if (status.motion.state == BODY_MOTION_CALIBRATING) {
            stateWrite = Menu_AppendText(stateWrite, stateEnd, "CAL ");
            stateWrite = Menu_AppendUnsigned(stateWrite, stateEnd,
                status.motion.calibrationCount);
            stateWrite = Menu_AppendChar(stateWrite, stateEnd, '/');
            (void)Menu_AppendUnsigned(stateWrite, stateEnd,
                status.motion.calibrationTarget);
        } else if (status.motion.state == BODY_MOTION_READY) {
            stateWrite = Menu_AppendText(stateWrite, stateEnd,
                (status.holdEnabled != 0U) ? "HOLD" : "OBSERVE");
            stateWrite = Menu_AppendText(stateWrite, stateEnd, " FF");
            (void)Menu_AppendUnsigned(stateWrite, stateEnd,
                status.feedForwardEnabled);
        } else if (status.motion.state == BODY_MOTION_STALE) {
            (void)Menu_AppendText(stateWrite, stateEnd, "IMU STALE");
        } else {
            (void)Menu_AppendText(stateWrite, stateEnd, "WAIT IMU");
        }
        yawWrite = Menu_AppendText(yawWrite, yawEnd, "Y100 ");
        yawWrite = Menu_AppendSigned(yawWrite, yawEnd,
            status.motion.yawEstimateX100);
        yawWrite = Menu_AppendText(yawWrite, yawEnd, " R ");
        (void)Menu_AppendSigned(yawWrite, yawEnd,
            status.motion.yawRateFilteredX100PerSec);
        controlWrite = Menu_AppendText(controlWrite, controlEnd, "E ");
        controlWrite = Menu_AppendSigned(controlWrite, controlEnd,
            status.stepError);
        controlWrite = Menu_AppendText(controlWrite, controlEnd, " S ");
        (void)Menu_AppendSigned(controlWrite, controlEnd, status.commandSps);
        Menu_RenderLines("Task 8 IMU", stateLine, yawLine, controlLine);
        return;
    }

    if (missionId != 1U) {
        Menu_RenderLines(line, "", "", "");
        return;
    }

    targetLaps = StateMachine_GetMission1LapCount();
    turnCount = MotorNoYaw_GetTurnCount();
    targetTurns = (uint32_t)targetLaps * 4U;

    write = lapLine;
    end = &lapLine[MENU_LINE_SIZE - 1U];
    write = Menu_AppendText(write, end, "Lap ");
    write = Menu_AppendUnsigned(write, end, turnCount / 4U);
    write = Menu_AppendChar(write, end, '/');
    (void)Menu_AppendUnsigned(write, end, targetLaps);

    write = turnLine;
    end = &turnLine[MENU_LINE_SIZE - 1U];
    write = Menu_AppendText(write, end, "Turn ");
    write = Menu_AppendUnsigned(write, end, turnCount);
    write = Menu_AppendChar(write, end, '/');
    (void)Menu_AppendUnsigned(write, end, targetTurns);

    Menu_RenderLines(line, lapLine, turnLine, "NO YAW");
}

static void Menu_RenderSimple(const char *title)
{
    Menu_RenderLines(title, "", "", "");
}

/* 作用：Task1 完成后保留圈数和转向计数，便于区分真完成与异常进入 Finished。 */
static void Menu_RenderTask1Finished(void)
{
    char lapLine[MENU_LINE_SIZE];
    char turnLine[MENU_LINE_SIZE];
    char *write;
    char *end;
    uint8_t targetLaps = StateMachine_GetMission1LapCount();
    uint32_t turnCount = MotorNoYaw_GetTurnCount();
    uint32_t targetTurns = (uint32_t)targetLaps * 4U;

    write = lapLine;
    end = &lapLine[MENU_LINE_SIZE - 1U];
    write = Menu_AppendText(write, end, "Lap ");
    write = Menu_AppendUnsigned(write, end, turnCount / 4U);
    write = Menu_AppendChar(write, end, '/');
    (void)Menu_AppendUnsigned(write, end, targetLaps);

    write = turnLine;
    end = &turnLine[MENU_LINE_SIZE - 1U];
    write = Menu_AppendText(write, end, "Turn ");
    write = Menu_AppendUnsigned(write, end, turnCount);
    write = Menu_AppendChar(write, end, '/');
    (void)Menu_AppendUnsigned(write, end, targetTurns);

    Menu_RenderLines("Finished", "Task 1", lapLine, turnLine);
}

static void Menu_RenderByState(CarState state)
{
    switch (state) {
    case CAR_STATE_MENU:
        if (g_menuPage == MENU_PAGE_TASK1_LAPS) {
            Menu_RenderTask1LapMenu();
        } else if (g_menuPage == MENU_PAGE_TASK2_DISTANCE) {
            Menu_RenderGimbalDistanceMenu("Task 2", g_task2Distance);
        } else if (g_menuPage == MENU_PAGE_TASK3_DISTANCE) {
            Menu_RenderGimbalDistanceMenu("Task 3", g_task3Distance);
        } else if (g_menuPage == MENU_PAGE_TASK7_DISTANCE) {
            Menu_RenderGimbalDistanceMenu("Task 7 Circle", g_task7Distance);
        } else if (g_menuPage == MENU_PAGE_TASK6_MODE) {
            Menu_RenderDriveModeMenu(6U);
        } else if (g_menuPage == MENU_PAGE_TASK6_SPEED) {
            Menu_RenderDriveSpeedMenu(6U);
        } else {
            Menu_RenderTaskMenu();
        }
        break;
    case CAR_STATE_MISSION:
        Menu_RenderMission();
        break;
    case CAR_STATE_FINISHED:
        if (StateMachine_GetMissionId() == 1U) {
            Menu_RenderTask1Finished();
        } else {
            Menu_RenderSimple("Finished");
        }
        break;
    case CAR_STATE_STOP:
        Menu_RenderSimple("Stop");
        break;
    case CAR_STATE_ERROR:
        Menu_RenderSimple("Error");
        break;
    case CAR_STATE_INIT:
    default:
        Menu_RenderSimple("Init");
        break;
    }
}

void Menu_Init(void)
{
    g_menuPage = MENU_PAGE_MAIN;
    g_taskIndex = 0U;
    g_task1LapCount = 1U;
    g_task2Distance = 0U;
    g_task3Distance = 0U;
    g_task7Distance = 0U;
    g_task6DriveMode = (CHASSIS_TASK6_DEFAULT_CLOSED_LOOP != 0U) ?
        CAR_CHASSIS_DRIVE_CLOSED_LOOP : CAR_CHASSIS_DRIVE_OPEN_LOOP;
    g_task6ClosedSpeed = (uint16_t)CHASSIS_TASK6_CLOSED_SPEED_DEFAULT;
    g_task6OpenSpeed = (uint16_t)CHASSIS_TASK6_OPEN_SPEED_DEFAULT;
    g_forceRefresh = 1U;
    g_refreshTicks = CAR_MENU_REFRESH_TICKS;
    g_lastState = CAR_STATE_INIT;
}

void Menu_Next(void)
{
    if (g_menuPage == MENU_PAGE_TASK1_LAPS) {
        ++g_task1LapCount;
        if (g_task1LapCount > 5U) {
            g_task1LapCount = 1U;
        }
    } else if (g_menuPage == MENU_PAGE_TASK2_DISTANCE) {
        g_task2Distance = (uint8_t)((g_task2Distance + 1U) %
            MENU_GIMBAL_DISTANCE_COUNT);
    } else if (g_menuPage == MENU_PAGE_TASK3_DISTANCE) {
        g_task3Distance = (uint8_t)((g_task3Distance + 1U) %
            MENU_GIMBAL_DISTANCE_COUNT);
    } else if (g_menuPage == MENU_PAGE_TASK7_DISTANCE) {
        g_task7Distance = (uint8_t)((g_task7Distance + 1U) %
            MENU_GIMBAL_DISTANCE_COUNT);
    } else if (g_menuPage == MENU_PAGE_TASK6_MODE) {
        g_task6DriveMode = (g_task6DriveMode ==
            CAR_CHASSIS_DRIVE_CLOSED_LOOP) ?
            CAR_CHASSIS_DRIVE_OPEN_LOOP : CAR_CHASSIS_DRIVE_CLOSED_LOOP;
    } else if (g_menuPage == MENU_PAGE_TASK6_SPEED) {
        if (g_task6DriveMode == CAR_CHASSIS_DRIVE_CLOSED_LOOP) {
            g_task6ClosedSpeed = Menu_NextDriveSpeed(g_task6ClosedSpeed);
        } else {
            g_task6OpenSpeed = Menu_NextDriveSpeed(g_task6OpenSpeed);
        }
    } else {
        g_taskIndex = (uint8_t)((g_taskIndex + 1U) % MENU_TASK_COUNT);
    }
    Menu_RequestRefresh();
}

CarEvent Menu_Confirm(void)
{
    if (g_menuPage == MENU_PAGE_TASK1_LAPS) {
        StateMachine_SetMission1LapCount(g_task1LapCount);
        return CAR_EVENT_MISSION_1_START;
    }
    if (g_menuPage == MENU_PAGE_TASK2_DISTANCE) {
        StateMachine_SetMission2Distance(g_task2Distance);
        return CAR_EVENT_MISSION_2_START;
    }
    if (g_menuPage == MENU_PAGE_TASK3_DISTANCE) {
        StateMachine_SetMission3Distance(g_task3Distance);
        return CAR_EVENT_MISSION_3_START;
    }
    if (g_menuPage == MENU_PAGE_TASK7_DISTANCE) {
        StateMachine_SetMission7Distance(g_task7Distance);
        return CAR_EVENT_MISSION_7_START;
    }
    if (g_menuPage == MENU_PAGE_TASK6_MODE) {
        g_menuPage = MENU_PAGE_TASK6_SPEED;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    }
    if (g_menuPage == MENU_PAGE_TASK6_SPEED) {
        StateMachine_SetMissionDriveConfig(6U, g_task6DriveMode,
            Menu_GetDriveSpeed(6U));
        return CAR_EVENT_MISSION_6_START;
    }
    switch (g_taskIndex) {
    case 0U:
        g_menuPage = MENU_PAGE_TASK1_LAPS;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    case 1U:
        g_menuPage = MENU_PAGE_TASK2_DISTANCE;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    case 2U:
        g_menuPage = MENU_PAGE_TASK3_DISTANCE;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    case 3U:
        return CAR_EVENT_MISSION_4_START;
    case 4U:
        return CAR_EVENT_MISSION_5_START;
    case 5U:
        g_menuPage = MENU_PAGE_TASK6_MODE;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    case 6U:
        g_menuPage = MENU_PAGE_TASK7_DISTANCE;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    case 7U:
        return CAR_EVENT_MISSION_8_START;
    default:
        return CAR_EVENT_NONE;
    }
}

uint8_t Menu_Back(void)
{
    if (g_menuPage == MENU_PAGE_MAIN) {
        return 0U;
    }

    if (g_menuPage == MENU_PAGE_TASK6_SPEED) {
        g_menuPage = MENU_PAGE_TASK6_MODE;
    } else {
        g_menuPage = MENU_PAGE_MAIN;
    }
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
        ((StateMachine_GetMissionId() == 5U) ||
            (StateMachine_GetMissionId() == 8U))) {
        refreshTicks = MENU_TASK5_REFRESH_TICKS;
    }
    if (state != g_lastState) {
        if (state == CAR_STATE_MENU) {
            g_menuPage = MENU_PAGE_MAIN;
        }
        g_lastState = state;
        g_forceRefresh = 1U;
        g_refreshTicks = refreshTicks;
    } else if (g_refreshTicks < refreshTicks) {
        ++g_refreshTicks;
    }

    if ((g_forceRefresh == 0U) &&
        (g_refreshTicks < refreshTicks)) {
        return;
    }

    g_forceRefresh = 0U;
    g_refreshTicks = 0U;
    Menu_RenderByState(state);
    if (Board_IsOledAvailable() == 0U) {
        g_forceRefresh = 1U;
    }
}
