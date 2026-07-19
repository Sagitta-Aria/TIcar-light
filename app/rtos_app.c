#include "rtos_app.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "app.h"
#include "board_config.h"
#include "control_config.h"
#include "menu.h"
#include "motor.h"
#include "state_machine.h"
#include "vision.h"

/*
 * RTOS 应用调度总览
 *
 * 1. 所有任务在启动时一次性创建，此后永久存在；比赛 Task1~9 只是
 *    StateMachine 的业务状态，不是 FreeRTOS 任务。
 * 2. CarControl/Gimbal 是最高优先级实时控制任务；Input/Mission 处理
 *    离散事件；Comm/UI 在较低优先级执行通信维护和显示。
 * 3. 直接任务通知相当于轻量“唤醒铃”：只表示有工作，不保存事件内容。
 *    必须逐个处理的 CarEvent 单独放入 g_eventQueue，防止事件被合并。
 * 4. 周期任务使用 xTaskDelayUntil 或带超时的 ulTaskNotifyTake，空闲时均
 *    处于阻塞态；CPU 最终运行 FreeRTOS Idle 任务，而不是轮询所有任务。
 */
#define RTOS_EVENT_QUEUE_LENGTH          (8U)

/* FreeRTOS 数值越大优先级越高；configMAX_PRIORITIES=7，合法范围为 0~6。 */
#define RTOS_CONTROL_PRIORITY            (6U)
#define RTOS_GIMBAL_PRIORITY             (6U)
#define RTOS_INPUT_PRIORITY              (4U)
#define RTOS_MISSION_PRIORITY            (3U)
#define RTOS_COMM_PRIORITY               (2U)
#define RTOS_UI_PRIORITY                 (1U)

#define RTOS_HOUSEKEEPING_PERIOD_MS      (20U)
#define RTOS_DYNAMIC_UI_PERIOD_MS        (100U)
#define RTOS_UI_STALE_CHECKS             \
    ((CAR_WATCHDOG_UI_TIMEOUT_MS + CAR_WATCHDOG_CHECK_PERIOD_MS - 1U) / \
        CAR_WATCHDOG_CHECK_PERIOD_MS)

/* 栈深度单位是 StackType_t（本 Cortex-M0+ 工程中为 32 位字），不是字节。 */
#define RTOS_CONTROL_STACK_WORDS         (128U)
#define RTOS_GIMBAL_STACK_WORDS          (256U)
#define RTOS_INPUT_STACK_WORDS           (128U)
#define RTOS_MISSION_STACK_WORDS         (192U)
#define RTOS_COMM_STACK_WORDS            (256U)
#define RTOS_UI_STACK_WORDS              (384U)

/* StaticTask_t 保存内核任务控制块；实际任务栈由下一组数组提供。 */
static StaticTask_t g_controlTaskControl;
static StaticTask_t g_gimbalTaskControl;
static StaticTask_t g_inputTaskControl;
#if CAR_ENABLE_UI_WATCHDOG
static StaticTask_t g_watchdogTaskControl;
#endif
static StaticTask_t g_missionTaskControl;
static StaticTask_t g_commTaskControl;
static StaticTask_t g_uiTaskControl;

static StackType_t g_controlTaskStack[RTOS_CONTROL_STACK_WORDS];
static StackType_t g_gimbalTaskStack[RTOS_GIMBAL_STACK_WORDS];
static StackType_t g_inputTaskStack[RTOS_INPUT_STACK_WORDS];
#if CAR_ENABLE_UI_WATCHDOG
static StackType_t g_watchdogTaskStack[CAR_WATCHDOG_TASK_STACK_WORDS];
#endif
static StackType_t g_missionTaskStack[RTOS_MISSION_STACK_WORDS];
static StackType_t g_commTaskStack[RTOS_COMM_STACK_WORDS];
static StackType_t g_uiTaskStack[RTOS_UI_STACK_WORDS];

/*
 * 队列只传递 CarEvent；TaskHandle_t 用于直接通知或挂起指定任务。
 * volatile 标志会跨任务访问，但实际修改点都由任务调度顺序约束。
 */
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
static volatile uint8_t g_controlScheduleReset;
#if CAR_ENABLE_UI_WATCHDOG
static volatile uint32_t g_uiHeartbeat;
#endif

static volatile const char *g_assertFile;
static volatile int g_assertLine;

static uint8_t RtosApp_ShouldSuspendControl(void);

#if CAR_ENABLE_UI_WATCHDOG
#if (CAR_WATCHDOG_CHECK_PERIOD_MS == 0U)
#error "CAR_WATCHDOG_CHECK_PERIOD_MS must be greater than zero"
#endif

/* WWDT0分频和周期由board_config.h配置；调试暂停时同时暂停看门狗。 */
static void RtosApp_InitWatchdog(void)
{
    DL_WWDT_reset(WWDT0);
    DL_WWDT_enablePower(WWDT0);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_WWDT_initWatchdogMode(WWDT0, CAR_WATCHDOG_HW_CLOCK_DIVIDER,
        CAR_WATCHDOG_HW_TIMER_PERIOD, DL_WWDT_RUN_IN_SLEEP,
        DL_WWDT_WINDOW_PERIOD_0, DL_WWDT_WINDOW_PERIOD_0);
    DL_WWDT_setActiveWindow(WWDT0, DL_WWDT_WINDOW0);
    DL_WWDT_setCoreHaltBehavior(WWDT0, DL_WWDT_CORE_HALT_STOP);
    DL_WWDT_restart(WWDT0);
}

/* UI心跳达到配置超时后停止喂狗，让WWDT0复位整机。 */
static void RtosApp_WatchdogTask(void *parameter)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    uint32_t lastHeartbeat = g_uiHeartbeat;
    uint32_t currentHeartbeat;
    uint32_t staleChecks = 0U;
    uint8_t healthy = 1U;

    (void)parameter;
    for (;;) {
        (void)xTaskDelayUntil(&lastWakeTime,
            pdMS_TO_TICKS(CAR_WATCHDOG_CHECK_PERIOD_MS));
        currentHeartbeat = g_uiHeartbeat;
        if (currentHeartbeat != lastHeartbeat) {
            lastHeartbeat = currentHeartbeat;
            staleChecks = 0U;
        } else if (staleChecks < RTOS_UI_STALE_CHECKS) {
            ++staleChecks;
        }

        if (staleChecks >= RTOS_UI_STALE_CHECKS) {
            healthy = 0U;
        }
        if (healthy != 0U) {
            DL_WWDT_restart(WWDT0);
        }
    }
}
#endif

/* 控制任务可被快事件提前唤醒，但周期控制仍必须等满 20 ms 截止时间。 */
static TickType_t RtosApp_GetControlWaitTicks(TickType_t lastControlTime)
{
    TickType_t period = pdMS_TO_TICKS(CHASSIS_CONTROL_PERIOD_MS);
    TickType_t elapsed = xTaskGetTickCount() - lastControlTime;

    return (elapsed >= period) ? 0U : period - elapsed;
}

/*
 * CarControl（优先级 6）：
 * - 常规路径每 CHASSIS_CONTROL_PERIOD_MS（当前 20 ms）执行一次底盘控制；
 * - TIMG0 识别到转弯/回线语义事件时可通过通知提前唤醒，只处理快事件，
 *   不会提前读取并清零编码器窗口，也不会提前运行速度 PI；
 * - Task2/3/7/8 期间由 Mission 挂起，恢复时重置周期基准，避免补跑旧周期。
 */
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

/*
 * Gimbal（优先级 6）：完整视觉帧通知到达时立即抢占执行；没有通知时
 * 最多等待 BODY_MOTION_PERIOD_MS（当前 10 ms），保证姿态矫正持续更新。
 */
static void RtosApp_GimbalTask(void *parameter)
{
    uint32_t frameCount;

    (void)parameter;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE,
            pdMS_TO_TICKS(BODY_MOTION_PERIOD_MS));
        frameCount = Vision_GetFrameCount();
        App_GimbalStep();
        if (Vision_GetFrameCount() != frameCount) {
            RtosApp_NotifyMission();
            RtosApp_NotifyUi();
        }
    }
}

/* 纯云台任务和Task9手推测试不需要底盘周期，由Mission统一挂起CarControl。 */
static uint8_t RtosApp_ShouldSuspendControl(void)
{
    uint8_t missionId;

    if (StateMachine_GetState() != CAR_STATE_MISSION) {
        return 0U;
    }

    missionId = StateMachine_GetMissionId();
    return (uint8_t)((missionId == 2U) || (missionId == 3U) ||
        (missionId == 7U) || (missionId == 8U) || (missionId == 9U));
}

/* Task2/3/7/8是纯云台任务；Task9只采编码器，运行时停止底盘控制调度。 */
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

/*
 * Input（优先级 4）：GPIO 按键边沿从 ISR 唤醒本任务。任务被唤醒后每
 * 1 ms 执行消抖/长按检测，直到按键释放且事件取空，再无限期阻塞。
 * UI 只需一次刷新通知；状态切换事件则进入队列并唤醒 Mission。
 */
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

/* Mission 通常由事件唤醒；Task3 准备期和 Task4 非循迹阶段需要 1 ms 步进。 */
static uint8_t RtosApp_IsMissionPeriodic(void)
{
    uint8_t missionId;

    if (StateMachine_GetState() != CAR_STATE_MISSION) {
        return 0U;
    }
    missionId = StateMachine_GetMissionId();
    if (missionId == 3U) {
        return (uint8_t)(StateMachine_IsMissionGimbalPrepDone() == 0U);
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

/*
 * Mission（优先级 3）：先排空 CarEvent 队列并驱动状态切换，再按当前比赛
 * 阶段决定是否执行 1 ms 周期步骤，最后同步 CarControl 的挂起/恢复状态。
 * 通知只负责唤醒；真正不能丢的事件均从队列读取。
 */
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
        if (handledEvent != 0U) {
            RtosApp_NotifyUi();
        }
    }
}

/* Comm（优先级 2）：每 5 ms 维护外部 Link、串口调参和低频日志。 */
static void RtosApp_CommTask(void *parameter)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    (void)parameter;
    for (;;) {
        App_CommStep();
        (void)xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(5U));
    }
}

/*
 * UI（优先级 1）：状态变化通知可立即触发重绘，同时最多每 20 ms 醒来做
 * Board housekeeping；Task5/6/8/9动态页另以100 ms限速刷新。显示和板级
 * 恢复都放在最低业务优先级，避免阻塞控制任务。
 */
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
#if CAR_ENABLE_UI_WATCHDOG
        ++g_uiHeartbeat;
#endif
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
            if (App_HousekeepingStep() != 0U) {
                App_UiStep();
            }
        }

        dynamicUi = (uint8_t)((StateMachine_GetState() == CAR_STATE_MISSION) &&
            ((StateMachine_GetMissionId() == 5U) ||
                (StateMachine_GetMissionId() == 6U) ||
                (StateMachine_GetMissionId() == 8U) ||
                (StateMachine_GetMissionId() == 9U)));
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

/*
 * 启动前一次性创建队列、任务控制块和任务栈，全程不使用 FreeRTOS 堆。
 * 末尾的四次通知让事件驱动任务在调度器启动后至少运行一次，建立初始
 * 输入、状态机、云台和 UI 状态；Comm/CarControl 本身已有周期唤醒路径。
 */
static void RtosApp_CreateObjects(void)
{
    g_controlTaskSuspended = 0U;
    g_controlScheduleReset = 0U;
#if CAR_ENABLE_UI_WATCHDOG
    g_uiHeartbeat = 0U;
#endif
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
#if CAR_ENABLE_UI_WATCHDOG
    configASSERT(xTaskCreateStatic(RtosApp_WatchdogTask, "Watchdog",
        CAR_WATCHDOG_TASK_STACK_WORDS, 0, CAR_WATCHDOG_TASK_PRIORITY,
        g_watchdogTaskStack, &g_watchdogTaskControl) != 0);
#endif
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
#if CAR_ENABLE_UI_WATCHDOG
    RtosApp_InitWatchdog();
#endif
}

/* 任务上下文通知：计数可合并，接收方 ulTaskNotifyTake(pdTRUE, ...) 会清零。 */
static void RtosApp_NotifyTask(TaskHandle_t task)
{
    if (task != 0) {
        xTaskNotifyGive(task);
    }
}

/* ISR 通知后若有更高优先级任务就绪，要求在中断退出点立即完成抢占。 */
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

/* vTaskStartScheduler() 成功后不会返回；返回只能表示内核启动失败。 */
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
