#include "rtos_app.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "app.h"
#include "board_config.h"
#include "control_config.h"
#include "gimbal.h"
#include "menu.h"
#include "motor.h"
#include "state_machine.h"
#include "vision.h"

#define RTOS_EVENT_QUEUE_LENGTH          (8U)

#define RTOS_CONTROL_PRIORITY            (6U)
#define RTOS_GIMBAL_PRIORITY             (6U)
#define RTOS_INPUT_PRIORITY              (4U)
#define RTOS_MISSION_PRIORITY            (3U)
#define RTOS_COMM_PRIORITY               (2U)
#define RTOS_UI_PRIORITY                 (1U)

#define RTOS_HOUSEKEEPING_PERIOD_MS      (20U)
#define RTOS_DYNAMIC_UI_PERIOD_MS        (100U)

#define RTOS_CONTROL_STACK_WORDS         (128U)
#define RTOS_GIMBAL_STACK_WORDS          (192U)
#define RTOS_INPUT_STACK_WORDS           (128U)
#define RTOS_MISSION_STACK_WORDS         (192U)
#define RTOS_COMM_STACK_WORDS            (256U)
#define RTOS_UI_STACK_WORDS              (384U)

static StaticTask_t g_controlTaskControl;
static StaticTask_t g_gimbalTaskControl;
static StaticTask_t g_inputTaskControl;
static StaticTask_t g_missionTaskControl;
static StaticTask_t g_commTaskControl;
static StaticTask_t g_uiTaskControl;

static StackType_t g_controlTaskStack[RTOS_CONTROL_STACK_WORDS];
static StackType_t g_gimbalTaskStack[RTOS_GIMBAL_STACK_WORDS];
static StackType_t g_inputTaskStack[RTOS_INPUT_STACK_WORDS];
static StackType_t g_missionTaskStack[RTOS_MISSION_STACK_WORDS];
static StackType_t g_commTaskStack[RTOS_COMM_STACK_WORDS];
static StackType_t g_uiTaskStack[RTOS_UI_STACK_WORDS];

static StaticQueue_t g_eventQueueControl;
static uint8_t g_eventQueueStorage[
    RTOS_EVENT_QUEUE_LENGTH * sizeof(CarEvent)];
static QueueHandle_t g_eventQueue;
static TaskHandle_t g_controlTaskHandle;
static TaskHandle_t g_gimbalTaskHandle;
static TaskHandle_t g_inputTaskHandle;
static TaskHandle_t g_missionTaskHandle;
static TaskHandle_t g_uiTaskHandle;
static uint8_t g_controlTaskSuspended;
static uint8_t g_gimbalTaskSuspended;
static volatile uint8_t g_controlScheduleReset;

static volatile const char *g_assertFile;
static volatile int g_assertLine;

static uint8_t RtosApp_ShouldSuspendControl(void);

static TickType_t RtosApp_GetControlWaitTicks(TickType_t lastControlTime)
{
    TickType_t period = pdMS_TO_TICKS(CHASSIS_CONTROL_PERIOD_MS);
    TickType_t elapsed = xTaskGetTickCount() - lastControlTime;

    return (elapsed >= period) ? 0U : period - elapsed;
}

static void RtosApp_ControlTask(void *parameter)
{
    TickType_t lastControlTime = xTaskGetTickCount();
    TickType_t now;
    TickType_t period = pdMS_TO_TICKS(CHASSIS_CONTROL_PERIOD_MS);
    uint32_t notified;

    (void)parameter;
    for (;;) {
        notified = ulTaskNotifyTake(pdTRUE,
            RtosApp_GetControlWaitTicks(lastControlTime));
        now = xTaskGetTickCount();
        if (g_controlScheduleReset != 0U) {
            g_controlScheduleReset = 0U;
            lastControlTime = now;
            continue;
        }

        if (RtosApp_ShouldSuspendControl() != 0U) {
            lastControlTime = now;
            continue;
        }

        if (notified != 0U) {
            StateMachine_HandleChassisFastEvent();
        }

        now = xTaskGetTickCount();
        if ((now - lastControlTime) < period) {
            continue;
        }
        lastControlTime = now;
        StateMachine_ChassisControlPeriod();
        Motor_RunChassisControl();
    }
}

static void RtosApp_GimbalTask(void *parameter)
{
    TickType_t waitTicks;
    uint32_t frameCount;

    (void)parameter;
    for (;;) {
        waitTicks = (Gimbal_NeedsTimeoutService() != 0U) ?
            pdMS_TO_TICKS(CAR_GIMBAL_VISION_TIMEOUT_TICKS) :
            portMAX_DELAY;
        (void)ulTaskNotifyTake(pdTRUE, waitTicks);
        frameCount = Vision_GetFrameCount();
        App_GimbalStep();
        if (Vision_GetFrameCount() != frameCount) {
            RtosApp_NotifyMission();
            RtosApp_NotifyUi();
        }
    }
}

static uint8_t RtosApp_ShouldSuspendGimbal(void)
{
    return (uint8_t)((StateMachine_GetState() == CAR_STATE_MISSION) &&
        ((StateMachine_GetMissionId() == 1U) ||
            (StateMachine_GetMissionId() == 5U) ||
            (StateMachine_GetMissionId() == 6U) ||
            (StateMachine_GetMissionId() == 7U)));
}

static uint8_t RtosApp_ShouldSuspendControl(void)
{
    return (uint8_t)((StateMachine_GetState() == CAR_STATE_MISSION) &&
        (StateMachine_GetMissionId() == 7U) &&
        (StateMachine_GetMissionDriveMode(7U) ==
            CAR_CHASSIS_DRIVE_OPEN_LOOP));
}

/* Task7 开环由硬件保持 PWM；闭环必须保留 20ms CarControl。 */
static void RtosApp_UpdateControlTaskState(void)
{
    uint8_t shouldSuspend = RtosApp_ShouldSuspendControl();

    if ((shouldSuspend != 0U) && (g_controlTaskSuspended == 0U)) {
        g_controlTaskSuspended = 1U;
        vTaskSuspend(g_controlTaskHandle);
    } else if ((shouldSuspend == 0U) &&
        (g_controlTaskSuspended != 0U)) {
        g_controlTaskSuspended = 0U;
        g_controlScheduleReset = 1U;
        vTaskResume(g_controlTaskHandle);
    }
}

static void RtosApp_UpdateGimbalTaskState(void)
{
    uint8_t shouldSuspend = RtosApp_ShouldSuspendGimbal();

    if ((shouldSuspend != 0U) && (g_gimbalTaskSuspended == 0U)) {
        Gimbal_SetEnabled(0U);
        g_gimbalTaskSuspended = 1U;
        vTaskSuspend(g_gimbalTaskHandle);
    } else if ((shouldSuspend == 0U) &&
        (g_gimbalTaskSuspended != 0U)) {
        g_gimbalTaskSuspended = 0U;
        vTaskResume(g_gimbalTaskHandle);
        RtosApp_NotifyGimbal();
    }
}

static void RtosApp_InputTask(void *parameter)
{
    CarEvent event;
    TickType_t lastServiceTime;

    (void)parameter;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        lastServiceTime = xTaskGetTickCount();
        do {
            event = App_InputStep();
            if (App_InputHadEvent() != 0U) {
                RtosApp_NotifyUi();
            }
            if ((event != CAR_EVENT_NONE) &&
                (xQueueSendToBack(g_eventQueue, &event, 0U) == pdPASS)) {
                RtosApp_NotifyMission();
            }
            if (App_InputIsActive() == 0U) {
                break;
            }
            (void)xTaskDelayUntil(&lastServiceTime, pdMS_TO_TICKS(1U));
        } while (1);
    }
}

static uint8_t RtosApp_IsMissionPeriodic(void)
{
    uint8_t missionId;

    if (StateMachine_GetState() != CAR_STATE_MISSION) {
        return 0U;
    }
    missionId = StateMachine_GetMissionId();
    if ((missionId == 3U) || (missionId == 7U)) {
        return 1U;
    }
    if ((missionId == 4U) &&
        (StateMachine_GetMission4Stage() != CAR_MISSION4_STAGE_LINE)) {
        return 1U;
    }
    return 0U;
}

static TickType_t RtosApp_GetMissionWaitTicks(TickType_t lastPeriodicTime)
{
    TickType_t elapsed;
    TickType_t period = pdMS_TO_TICKS(1U);

    if (RtosApp_IsMissionPeriodic() == 0U) {
        return portMAX_DELAY;
    }
    elapsed = xTaskGetTickCount() - lastPeriodicTime;
    return (elapsed >= period) ? 0U : period - elapsed;
}

static void RtosApp_MissionTask(void *parameter)
{
    CarEvent event;
    uint8_t handledEvent;
    uint8_t periodicBeforeWait;
    uint8_t periodicAfterEvents;
    TickType_t lastPeriodicTime = xTaskGetTickCount();
    TickType_t now;

    (void)parameter;
    for (;;) {
        periodicBeforeWait = RtosApp_IsMissionPeriodic();
        (void)ulTaskNotifyTake(pdTRUE,
            RtosApp_GetMissionWaitTicks(lastPeriodicTime));
        handledEvent = 0U;
        while (xQueueReceive(g_eventQueue, &event, 0U) == pdPASS) {
            App_MissionDispatch(event);
            handledEvent = 1U;
        }
        now = xTaskGetTickCount();
        periodicAfterEvents = RtosApp_IsMissionPeriodic();
        if ((periodicAfterEvents != 0U) && (periodicBeforeWait == 0U)) {
            lastPeriodicTime = now;
        } else if ((periodicAfterEvents != 0U) &&
            ((now - lastPeriodicTime) >= pdMS_TO_TICKS(1U))) {
            lastPeriodicTime = now;
            App_MissionStep();
        } else if (periodicAfterEvents == 0U) {
            lastPeriodicTime = now;
        }
        RtosApp_UpdateControlTaskState();
        RtosApp_UpdateGimbalTaskState();
        if (handledEvent != 0U) {
            RtosApp_NotifyUi();
        }
    }
}

static void RtosApp_CommTask(void *parameter)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    (void)parameter;
    for (;;) {
        App_CommStep();
        (void)xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(5U));
    }
}

static void RtosApp_UiTask(void *parameter)
{
    TickType_t lastHousekeepingTime = xTaskGetTickCount();
    TickType_t lastDynamicUiTime = lastHousekeepingTime;
    TickType_t now;
    TickType_t elapsed;
    TickType_t waitTicks;
    uint8_t dynamicUi;

    (void)parameter;
    for (;;) {
        now = xTaskGetTickCount();
        elapsed = now - lastHousekeepingTime;
        waitTicks = (elapsed >= pdMS_TO_TICKS(RTOS_HOUSEKEEPING_PERIOD_MS)) ?
            0U : pdMS_TO_TICKS(RTOS_HOUSEKEEPING_PERIOD_MS) - elapsed;
        if (ulTaskNotifyTake(pdTRUE, waitTicks) != 0U) {
            App_UiStep();
        }

        now = xTaskGetTickCount();
        if ((now - lastHousekeepingTime) >=
            pdMS_TO_TICKS(RTOS_HOUSEKEEPING_PERIOD_MS)) {
            lastHousekeepingTime = now;
            App_HousekeepingStep();
        }

        dynamicUi = (uint8_t)((StateMachine_GetState() == CAR_STATE_MISSION) &&
            ((StateMachine_GetMissionId() == 5U) ||
                (StateMachine_GetMissionId() == 6U) ||
                (StateMachine_GetMissionId() == 7U)));
        if (dynamicUi == 0U) {
            lastDynamicUiTime = now;
        } else if ((now - lastDynamicUiTime) >=
            pdMS_TO_TICKS(RTOS_DYNAMIC_UI_PERIOD_MS)) {
            lastDynamicUiTime = now;
            Menu_RequestRefresh();
            App_UiStep();
        }
    }
}

static void RtosApp_CreateObjects(void)
{
    g_controlTaskSuspended = 0U;
    g_gimbalTaskSuspended = 0U;
    g_controlScheduleReset = 0U;
    g_eventQueue = xQueueCreateStatic(RTOS_EVENT_QUEUE_LENGTH,
        sizeof(CarEvent), g_eventQueueStorage, &g_eventQueueControl);
    configASSERT(g_eventQueue != 0);

    g_controlTaskHandle = xTaskCreateStatic(RtosApp_ControlTask, "CarControl",
        RTOS_CONTROL_STACK_WORDS, 0, RTOS_CONTROL_PRIORITY,
        g_controlTaskStack, &g_controlTaskControl);
    configASSERT(g_controlTaskHandle != 0);
    g_gimbalTaskHandle = xTaskCreateStatic(RtosApp_GimbalTask, "Gimbal",
        RTOS_GIMBAL_STACK_WORDS, 0, RTOS_GIMBAL_PRIORITY,
        g_gimbalTaskStack, &g_gimbalTaskControl);
    configASSERT(g_gimbalTaskHandle != 0);
    g_inputTaskHandle = xTaskCreateStatic(RtosApp_InputTask, "Input",
        RTOS_INPUT_STACK_WORDS, 0, RTOS_INPUT_PRIORITY,
        g_inputTaskStack, &g_inputTaskControl);
    configASSERT(g_inputTaskHandle != 0);
    g_missionTaskHandle = xTaskCreateStatic(RtosApp_MissionTask, "Mission",
        RTOS_MISSION_STACK_WORDS, 0, RTOS_MISSION_PRIORITY,
        g_missionTaskStack, &g_missionTaskControl);
    configASSERT(g_missionTaskHandle != 0);
    configASSERT(xTaskCreateStatic(RtosApp_CommTask, "Comm",
        RTOS_COMM_STACK_WORDS, 0, RTOS_COMM_PRIORITY,
        g_commTaskStack, &g_commTaskControl) != 0);
    g_uiTaskHandle = xTaskCreateStatic(RtosApp_UiTask, "UI",
        RTOS_UI_STACK_WORDS, 0, RTOS_UI_PRIORITY,
        g_uiTaskStack, &g_uiTaskControl);
    configASSERT(g_uiTaskHandle != 0);

    xTaskNotifyGive(g_inputTaskHandle);
    xTaskNotifyGive(g_missionTaskHandle);
    xTaskNotifyGive(g_gimbalTaskHandle);
    xTaskNotifyGive(g_uiTaskHandle);
}

static void RtosApp_NotifyTask(TaskHandle_t task)
{
    if (task != 0) {
        xTaskNotifyGive(task);
    }
}

static void RtosApp_NotifyTaskFromISR(TaskHandle_t task)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if (task != 0) {
        vTaskNotifyGiveFromISR(task, &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

void RtosApp_NotifyGimbal(void)
{
    RtosApp_NotifyTask(g_gimbalTaskHandle);
}

void RtosApp_NotifyMission(void)
{
    RtosApp_NotifyTask(g_missionTaskHandle);
}

void RtosApp_NotifyUi(void)
{
    RtosApp_NotifyTask(g_uiTaskHandle);
}

void RtosApp_NotifyGimbalFromISR(void)
{
    RtosApp_NotifyTaskFromISR(g_gimbalTaskHandle);
}

void RtosApp_NotifyControlFromISR(void)
{
    RtosApp_NotifyTaskFromISR(g_controlTaskHandle);
}

void RtosApp_NotifyInputFromISR(void)
{
    RtosApp_NotifyTaskFromISR(g_inputTaskHandle);
}

void RtosApp_StartScheduler(void)
{
    RtosApp_CreateObjects();
    vTaskStartScheduler();
    App_FreeRtosAssert(__FILE__, __LINE__);
}

void App_FreeRtosAssert(const char *file, int line)
{
    g_assertFile = file;
    g_assertLine = line;
    (void)g_assertFile;
    (void)g_assertLine;
    __disable_irq();
    for (;;) {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *taskName)
{
    (void)task;
    (void)taskName;
    App_FreeRtosAssert(__FILE__, __LINE__);
}
