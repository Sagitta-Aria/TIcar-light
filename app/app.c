#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "board_config.h"
#include "body_motion.h"
#include "delay.h"
#include "gimbal.h"
#include "gimbal_attitude.h"
#include "h7_gyro_link.h"
#include "jy61p.h"
#include "key.h"
#include "link.h"
#include "log_uart.h"
#include "menu.h"
#include "motor.h"
#include "motor_no_yaw.h"
#include "oled.h"
#include "state_machine.h"
#include "staticconfig.h"
#include "tuning_console.h"
#include "vision.h"

#define APP_TASK2_OLED_FONT_SIZE        (12U)
#define APP_TASK2_OLED_START_X          (6U)
#define APP_TASK2_OLED_START_Y          (16U)
#define APP_TASK2_OLED_LINE_STEP        (12U)
#define APP_TASK2_OLED_MAX_CHARS        (18U)
#define APP_CAMERA_LASER_DELAY_MS       (1000U)

typedef enum {
    APP_TASK2_OLED_NONE = 0,
    APP_TASK2_OLED_WAITING,
    APP_TASK2_OLED_TRACKING,
    APP_TASK2_OLED_LINE
} AppTask2OledStatus;

typedef enum {
    APP_CAMERA_LASER_IDLE = 0,
    APP_CAMERA_LASER_WAIT_FRAME,
    APP_CAMERA_LASER_TRACKING_DELAY,
    APP_CAMERA_LASER_SENT
} AppCameraLaserStage;

/*
 * 作用：把两个实体按键翻译成比赛菜单事件。
 * 说明：K1 在菜单里切换任务/参数；K2 确认；长按 K2 停止或返回上一级。
 */
static CarEvent App_HandleKeyEvent(KeyEvent event)
{
    CarState state;

    if (event == KEY_EVENT_NONE) {
        return CAR_EVENT_NONE;
    }

    state = StateMachine_GetState();

    if (event == KEY_EVENT_2_LONG) {
        if (state == CAR_STATE_MENU) {
            (void)Menu_Back();
            return CAR_EVENT_NONE;
        } else if (state == CAR_STATE_MISSION) {
            return CAR_EVENT_STOP;
        } else {
            return CAR_EVENT_MENU;
        }
    }

    if (state == CAR_STATE_MENU) {
        if (event == KEY_EVENT_1) {
            Menu_Next();
        } else if (event == KEY_EVENT_2) {
            return Menu_Confirm();
        }
        return CAR_EVENT_NONE;
    }

    if (((state == CAR_STATE_STOP) || (state == CAR_STATE_FINISHED) ||
        (state == CAR_STATE_ERROR)) && (event == KEY_EVENT_2)) {
        return CAR_EVENT_MENU;
    }
    return CAR_EVENT_NONE;
}

/* 作用：Task1 跑 NO YAW 时走快路径，避免 OLED/视觉/云台任务拖慢 240ms 等待。 */
static uint8_t App_IsTask1NoYawRunning(void)
{
    return (uint8_t)((StateMachine_GetState() == CAR_STATE_MISSION) &&
        (StateMachine_GetMissionId() == 1U) &&
        (MotorNoYaw_IsRunning() != 0U));
}

/* 作用：Task2 打靶时走快路径，只保留视觉解析、云台闭环和电机输出。 */
static uint8_t App_IsTask2GimbalRunning(void)
{
    return (uint8_t)((StateMachine_GetState() == CAR_STATE_MISSION) &&
        (StateMachine_GetMissionId() == 2U) &&
        (Vision_IsRunning() != 0U));
}

/* 作用：Task7 复用 Task2 快路径，但读取圆点误差参数。 */
static uint8_t App_IsTask7GimbalRunning(void)
{
    return (uint8_t)((StateMachine_GetState() == CAR_STATE_MISSION) &&
        (StateMachine_GetMissionId() == 7U) &&
        (Vision_IsRunning() != 0U));
}

static uint8_t g_appFastMissionId;
static AppTask2OledStatus g_appTask2OledStatus;
static uint8_t g_appDirectGimbalCommandSent;
static uint8_t g_appInputHadEvent;
static uint8_t g_appLaserMissionId;
static TickType_t g_appLaserDelayStartTick;
static AppCameraLaserStage g_appLaserStage;

/* 作用：进入快路径时只刷一次 OLED，避免任务已经启动但屏幕还停在菜单。 */
static void App_ShowFastMissionOnce(uint8_t missionId)
{
    if (g_appFastMissionId == missionId) {
        return;
    }

    g_appFastMissionId = missionId;
    Menu_RequestRefresh();
    Menu_Task(StateMachine_GetState());
    if (Board_IsOledAvailable() == 0U) {
        g_appFastMissionId = 0U;
    }
}

static void App_ResetCameraLaserCommand(void)
{
    g_appLaserMissionId = 0U;
    g_appLaserDelayStartTick = 0U;
    g_appLaserStage = APP_CAMERA_LASER_IDLE;
}

/* 作用：Task7 收到第一帧有效视觉误差后，只向 K230 发一次 F。 */
static void App_UpdateDirectGimbalCommand(void)
{
    if (g_appDirectGimbalCommandSent != 0U) {
        return;
    }
    if (Vision_HasFrame() == 0U) {
        return;
    }

    Link_SendByte((uint8_t)'F');
    g_appDirectGimbalCommandSent = 1U;
}

/* 作用：Task2/Task3/Task4 收到首帧后非阻塞等待 1s，再发送一次 F。 */
static void App_UpdateCameraLaserCommand(uint8_t missionId)
{
    TickType_t now;

    if ((missionId != 2U) && (missionId != 3U) && (missionId != 4U)) {
        return;
    }

    if ((StateMachine_GetState() != CAR_STATE_MISSION) ||
        (StateMachine_GetMissionId() != missionId)) {
        App_ResetCameraLaserCommand();
        return;
    }

    if ((missionId != 2U) &&
        (StateMachine_IsMissionGimbalPrepDone() == 0U)) {
        return;
    }

    if (g_appLaserMissionId != missionId) {
        App_ResetCameraLaserCommand();
        g_appLaserMissionId = missionId;
        g_appLaserStage = APP_CAMERA_LASER_WAIT_FRAME;
    }

    if (g_appLaserStage == APP_CAMERA_LASER_SENT) {
        return;
    }

    if (g_appLaserStage == APP_CAMERA_LASER_IDLE) {
        g_appLaserStage = APP_CAMERA_LASER_WAIT_FRAME;
        return;
    }

    if (g_appLaserStage == APP_CAMERA_LASER_WAIT_FRAME) {
        if (Vision_HasFrame() == 0U) {
            return;
        }
        g_appLaserDelayStartTick = xTaskGetTickCount();
        g_appLaserStage = APP_CAMERA_LASER_TRACKING_DELAY;
        return;
    }

    if (g_appLaserStage != APP_CAMERA_LASER_TRACKING_DELAY) {
        return;
    }

    now = xTaskGetTickCount();
    if ((now - g_appLaserDelayStartTick) <
        pdMS_TO_TICKS(APP_CAMERA_LASER_DELAY_MS)) {
        return;
    }

    Link_SendByte((uint8_t)'F');
    g_appLaserStage = APP_CAMERA_LASER_SENT;
}

static void App_ShowTask2OledLine(uint8_t index, const char *text)
{
    char padded[APP_TASK2_OLED_MAX_CHARS + 1U];
    uint8_t i;
    uint8_t y;

    for (i = 0U; i < APP_TASK2_OLED_MAX_CHARS; ++i) {
        padded[i] = ' ';
    }
    padded[APP_TASK2_OLED_MAX_CHARS] = '\0';

    i = 0U;
    while ((text != 0) && (text[i] != '\0') &&
        (i < APP_TASK2_OLED_MAX_CHARS)) {
        padded[i] = text[i];
        ++i;
    }

    y = (uint8_t)(APP_TASK2_OLED_START_Y +
        (index * APP_TASK2_OLED_LINE_STEP));
    OLED_ShowString(APP_TASK2_OLED_START_X, y, (u8 *)padded,
        APP_TASK2_OLED_FONT_SIZE);
}

/* 作用：云台任务只在等待视觉和进入追踪时各刷一次 OLED。 */
static void App_ShowGimbalOledStatus(uint8_t missionId,
    AppTask2OledStatus status)
{
    char title[8];
    const char *statusText;

    if (Board_IsOledAvailable() == 0U) {
        return;
    }
    if (g_appTask2OledStatus == status) {
        return;
    }

    if (status == APP_TASK2_OLED_LINE) {
        statusText = "Line Follow";
    } else if (status == APP_TASK2_OLED_TRACKING) {
        statusText = "Tracking";
    } else {
        statusText = "Waiting K230";
    }
    title[0] = 'T';
    title[1] = 'a';
    title[2] = 's';
    title[3] = 'k';
    title[4] = ' ';
    title[5] = (char)('0' + missionId);
    title[6] = '\0';
    App_ShowTask2OledLine(0U, title);
    App_ShowTask2OledLine(1U, statusText);
    App_ShowTask2OledLine(2U, "");
    App_ShowTask2OledLine(3U, "");
    OLED_Refresh();
    if (Board_IsOledAvailable() != 0U) {
        g_appTask2OledStatus = status;
    }
}

static void App_ShowTask2OledStatus(AppTask2OledStatus status)
{
    App_ShowGimbalOledStatus(2U, status);
}

static void App_ShowTask3OledStatus(AppTask2OledStatus status)
{
    App_ShowGimbalOledStatus(3U, status);
}

static void App_ShowTask4OledStatus(AppTask2OledStatus status)
{
    App_ShowGimbalOledStatus(4U, status);
}

static void App_ShowTask7OledStatus(AppTask2OledStatus status)
{
    App_ShowGimbalOledStatus(7U, status);
}

/* 作用：Task3 打靶时走快路径，三档距离都直接追中心点。 */
static uint8_t App_IsTask3GimbalRunning(void)
{
    return (uint8_t)((StateMachine_GetState() == CAR_STATE_MISSION) &&
        (StateMachine_GetMissionId() == 3U) &&
        (Vision_IsRunning() != 0U));
}

/* 作用：Task4 同时跑视觉云台和 NO YAW，必须走快路径。 */
static uint8_t App_IsTask4Running(void)
{
    return (uint8_t)((StateMachine_GetState() == CAR_STATE_MISSION) &&
        (StateMachine_GetMissionId() == 4U) &&
        ((Vision_IsRunning() != 0U) ||
            (StateMachine_GetMission4Stage() != CAR_MISSION4_STAGE_IDLE)));
}

static void App_ClearFastMission(void)
{
    g_appFastMissionId = 0U;
    g_appTask2OledStatus = APP_TASK2_OLED_NONE;
    g_appDirectGimbalCommandSent = 0U;
    App_ResetCameraLaserCommand();
}

/*
 * 作用：初始化比赛正式版需要的 app 层模块。
 * 说明：这里只保留 NO YAW、云台闭环、视觉输入、菜单和状态机。
 */
void App_Init(void)
{
    Board_ShowBootProgress("I2C OK", "UART OK", "Gray OK", "APP...", "");
    LOG_LINE("app: init begin");
    delay_ms(100U);

    MotorNoYaw_Init();
    StaticConfig_Init();
    Gimbal_Init();
    Vision_Init();
    H7GyroLink_Init();
    JY61P_Init();
    BodyMotion_Init();
    GimbalAttitude_Init();
    Menu_Init();
    StateMachine_Init();

    LOG_LINE("m0-light-rtos competition init ok");
    Board_ShowBootProgress("I2C OK", "UART OK", "Gray OK", "APP OK", "");
    delay_ms(200U);

    if (Board_IsOledAvailable() != 0U) {
        OLED_Clear();
    }
    Menu_Task(StateMachine_GetState());
}

/*
 * 作用：应用层周期调度。
 * 说明：这些入口由不同FreeRTOS任务调用，不再由单一主循环串行调度。
 */
CarEvent App_InputStep(void)
{
    KeyEvent keyEvent;

    Key_Task();
    keyEvent = Key_PopEvent();
    g_appInputHadEvent = (keyEvent != KEY_EVENT_NONE) ? 1U : 0U;
    return App_HandleKeyEvent(keyEvent);
}

uint8_t App_InputHadEvent(void)
{
    return g_appInputHadEvent;
}

uint8_t App_InputIsActive(void)
{
    return (uint8_t)(((Key_IsPressed(KEY_ID_1) != 0U) ||
        (Key_IsPressed(KEY_ID_2) != 0U) ||
        (Key_HasPendingEvent() != 0U)) ? 1U : 0U);
}

void App_MissionDispatch(CarEvent event)
{
    if (event != CAR_EVENT_NONE) {
        StateMachine_Dispatch(event);
    }
}

void App_MissionStep(void)
{
    StateMachine_Task();
}

void App_CommStep(void)
{
    uint8_t missionId = StateMachine_GetMissionId();

    Link_Task();
    if ((StateMachine_GetState() == CAR_STATE_MISSION) &&
        (missionId == 7U)) {
        App_UpdateDirectGimbalCommand();
    } else {
        g_appDirectGimbalCommandSent = 0U;
    }
    if ((missionId == 2U) || (missionId == 3U) || (missionId == 4U)) {
        App_UpdateCameraLaserCommand(missionId);
    } else {
        App_ResetCameraLaserCommand();
    }
    LogUart_Task();
    TuningConsole_Task();
}

void App_GimbalStep(void)
{
    BodyMotion_Task();
    Vision_Task();
    if (GimbalAttitude_IsActive() != 0U) {
        if (GimbalAttitude_DrivesMotorDirectly() != 0U) {
            Gimbal_SetYawAttitudeCompensation(0);
            GimbalAttitude_SetReferenceTracking(0U);
            GimbalAttitude_Task();
        } else {
            /*
             * Task4主动转yaw时让H7目标跟随；主动命令停止后，H7重新锁定
             * 当前角度并把矫正速度叠加到视觉云台输出。
             */
            GimbalAttitude_SetReferenceTracking(
                Gimbal_IsYawTrackingActive());
            GimbalAttitude_Task();
            if (Gimbal_IsEnabled() != 0U) {
                Gimbal_SetYawAttitudeCompensation(
                    GimbalAttitude_GetCommandSps());
            } else {
                Gimbal_SetYawAttitudeCompensation(0);
            }
            Gimbal_Task();
        }
    } else {
        Gimbal_SetYawAttitudeCompensation(0);
        Gimbal_Task();
    }
    Motor_Task();
}

void App_UiStep(void)
{
    if (App_IsTask1NoYawRunning() != 0U) {
        App_ShowFastMissionOnce(1U);
    } else if (App_IsTask2GimbalRunning() != 0U) {
        App_ShowTask2OledStatus((Vision_GetFrameCount() == 0U) ?
            APP_TASK2_OLED_WAITING : APP_TASK2_OLED_TRACKING);
    } else if (App_IsTask3GimbalRunning() != 0U) {
        App_ShowTask3OledStatus((Vision_GetFrameCount() == 0U) ?
            APP_TASK2_OLED_WAITING : APP_TASK2_OLED_TRACKING);
    } else if (App_IsTask4Running() != 0U) {
        if (StateMachine_GetMission4Stage() == CAR_MISSION4_STAGE_LINE) {
            App_ShowTask4OledStatus(APP_TASK2_OLED_LINE);
        } else {
            App_ShowTask4OledStatus((Vision_GetFrameCount() == 0U) ?
                APP_TASK2_OLED_WAITING : APP_TASK2_OLED_TRACKING);
        }
    } else if (App_IsTask7GimbalRunning() != 0U) {
        App_ShowTask7OledStatus((Vision_GetFrameCount() == 0U) ?
            APP_TASK2_OLED_WAITING : APP_TASK2_OLED_TRACKING);
    } else {
        App_ClearFastMission();
        Menu_Task(StateMachine_GetState());
    }
}

uint8_t App_HousekeepingStep(void)
{
    if (Board_Task() == 0U) {
        return 0U;
    }

    g_appFastMissionId = 0U;
    g_appTask2OledStatus = APP_TASK2_OLED_NONE;
    Menu_RequestRefresh();
    return 1U;
}

void App_Task(void)
{
    CarEvent event = App_InputStep();

    App_MissionDispatch(event);
    App_MissionStep();
    App_GimbalStep();
    App_CommStep();
    App_UiStep();
    App_HousekeepingStep();
    delay_ms(CAR_APP_LOOP_DELAY_MS);
}
