#include "state_machine.h"

#include "gray.h"
#include "log_uart.h"
#include "motor_test.h"
#include "motion.h"
#include "route.h"
#include "tracking.h"
#include "tracking_exception.h"

static CarState g_carState = CAR_STATE_INIT;

static const char *StateMachine_GetEventName(CarEvent event)
{
    switch (event) {
    case CAR_EVENT_START:
        return "start";
    case CAR_EVENT_STOP:
        return "stop";
    case CAR_EVENT_MENU:
        return "menu";
    case CAR_EVENT_BACK:
        return "back";
    case CAR_EVENT_GRAY_CALIBRATION_START:
        return "gray_calibration_start";
    case CAR_EVENT_GRAY_CALIBRATION_SAMPLE:
        return "gray_calibration_sample";
    case CAR_EVENT_GRAY_CALIBRATION_APPLY:
        return "gray_calibration_apply";
    case CAR_EVENT_MOTOR_TEST_NEXT:
        return "motor_test_next";
    case CAR_EVENT_TRACKING_TEST_START:
        return "tracking_test_start";
    case CAR_EVENT_PID_MONITOR_START:
        return "pid_monitor_start";
    case CAR_EVENT_GRAY_MONITOR_START:
        return "gray_monitor_start";
    case CAR_EVENT_EXCHANGE_MONITOR_START:
        return "exchange_monitor_start";
    case CAR_EVENT_ENCODER_MONITOR_START:
        return "encoder_monitor_start";
    case CAR_EVENT_MISSION_START:
        return "mission_start";
    case CAR_EVENT_TRACKING_DONE:
        return "tracking_done";
    case CAR_EVENT_ERROR:
        return "error";
    case CAR_EVENT_CLEAR_ERROR:
        return "clear_error";
    case CAR_EVENT_NONE:
    default:
        return "none";
    }
}

/*
 * 作用：停止所有会让车运动的模块。
 * 使用场景：进入菜单、监视、校准、停止、错误等非运行状态时。
 */
static void StateMachine_StopMotionModules(void)
{
    Tracking_SetEnabled(0U);
    Route_Stop();
    MotorTest_Stop();
    Motion_Stop();
}

/*
 * 作用：进入空闲状态。
 * 使用场景：需要让车完全待机时。
 */
static void StateMachine_EnterIdle(void)
{
    StateMachine_StopMotionModules();
    LOG_LINE("state: idle");
}

/*
 * 作用：进入菜单状态。
 * 使用场景：OLED 选择界面，按键确认/切换都在 menu 模块处理。
 */
static void StateMachine_EnterMenu(void)
{
    StateMachine_StopMotionModules();
    LOG_LINE("state: menu");
}

/*
 * 作用：进入灰度校准状态。
 * 使用场景：菜单选择 1.Calib 后。
 * 说明：进入时重置最小/最大值，周期任务里持续采样。
 */
static void StateMachine_EnterGrayCalibration(void)
{
    StateMachine_StopMotionModules();
    Gray_CalibrationReset();
    LOG_LINE("state: gray calibration");
}

/*
 * 作用：进入正式循迹状态。
 * 使用场景：后续赛题任务或完整路线任务需要跑路线外环时。
 */
static void StateMachine_EnterTracking(void)
{
    Route_Start();
    MotorTest_Stop();
    TrackingException_Reset();
    Tracking_SetEnabled(1U);
    LOG_LINE("state: tracking");
}

/*
 * 作用：进入单纯循迹测试状态。
 * 使用场景：测试菜单里的 TrackOnly，只跑灰度循迹，不跑路线外环。
 */
static void StateMachine_EnterTrackingTest(void)
{
    Route_Stop();
    MotorTest_Stop();
    TrackingException_Reset();
    Tracking_SetEnabled(1U);
    LOG_LINE("state: tracking test");
}

/*
 * 作用：进入电机方向确认状态。
 * 使用场景：测试菜单里的 MotorDir。
 */
static void StateMachine_EnterMotorTest(void)
{
    Tracking_SetEnabled(0U);
    Route_Stop();
    MotorTest_Start();
    LOG_LINE("state: motor test");
}

/*
 * 作用：进入 PID 数据监视状态。
 * 使用场景：测试菜单里查看速度闭环输出和实际编码器增量。
 */
static void StateMachine_EnterPidMonitor(void)
{
    StateMachine_StopMotionModules();
    LOG_LINE("state: pid monitor");
}

/*
 * 作用：进入灰度数据监视状态。
 * 使用场景：测试菜单里查看灰度传感器原始值、数字量和误差。
 */
static void StateMachine_EnterGrayMonitor(void)
{
    StateMachine_StopMotionModules();
    LOG_LINE("state: gray monitor");
}

/*
 * 作用：进入 Exchange 数据监视状态。
 * 使用场景：后续视觉模块数据接入后查看通信内容。
 * 说明：现在只保留状态入口，不主动改动视觉模块协议。
 */
static void StateMachine_EnterExchangeMonitor(void)
{
    StateMachine_StopMotionModules();
    LOG_LINE("state: exchange monitor");
}

/*
 * 作用：进入编码器数据监视状态。
 * 使用场景：测试菜单里查看左右编码器计数。
 */
static void StateMachine_EnterEncoderMonitor(void)
{
    StateMachine_StopMotionModules();
    LOG_LINE("state: encoder monitor");
}

/*
 * 作用：进入赛题任务状态。
 * 使用场景：主菜单里的 Mission，具体赛题流程后续再填。
 */
static void StateMachine_EnterMission(void)
{
    StateMachine_StopMotionModules();
    LOG_LINE("state: mission placeholder");
}

/*
 * 作用：进入任务完成状态。
 * 使用场景：以后路线跑完、视觉任务完成或比赛流程结束。
 */
static void StateMachine_EnterFinished(void)
{
    StateMachine_StopMotionModules();
    LOG_LINE("state: finished");
}

/*
 * 作用：进入停止状态。
 * 使用场景：运行过程中主动停车或循迹异常无法恢复。
 */
static void StateMachine_EnterStop(void)
{
    StateMachine_StopMotionModules();
    LOG_LINE("state: stop");
}

/*
 * 作用：进入错误状态。
 * 使用场景：ADC 连续异常、后续硬件故障等。
 */
static void StateMachine_EnterError(void)
{
    StateMachine_StopMotionModules();
    LOG_LINE("state: error");
}

/*
 * 作用：统一处理状态切换入口动作。
 * 使用场景：任何事件触发状态变化时。
 */
static void StateMachine_Enter(CarState nextState)
{
    if (g_carState == nextState) {
        return;
    }

    g_carState = nextState;

    switch (g_carState) {
    case CAR_STATE_IDLE:
        StateMachine_EnterIdle();
        break;
    case CAR_STATE_MENU:
        StateMachine_EnterMenu();
        break;
    case CAR_STATE_GRAY_CALIBRATION:
        StateMachine_EnterGrayCalibration();
        break;
    case CAR_STATE_TRACKING:
        StateMachine_EnterTracking();
        break;
    case CAR_STATE_TRACKING_TEST:
        StateMachine_EnterTrackingTest();
        break;
    case CAR_STATE_MOTOR_TEST:
        StateMachine_EnterMotorTest();
        break;
    case CAR_STATE_PID_MONITOR:
        StateMachine_EnterPidMonitor();
        break;
    case CAR_STATE_GRAY_MONITOR:
        StateMachine_EnterGrayMonitor();
        break;
    case CAR_STATE_EXCHANGE_MONITOR:
        StateMachine_EnterExchangeMonitor();
        break;
    case CAR_STATE_ENCODER_MONITOR:
        StateMachine_EnterEncoderMonitor();
        break;
    case CAR_STATE_MISSION:
        StateMachine_EnterMission();
        break;
    case CAR_STATE_FINISHED:
        StateMachine_EnterFinished();
        break;
    case CAR_STATE_STOP:
        StateMachine_EnterStop();
        break;
    case CAR_STATE_ERROR:
        StateMachine_EnterError();
        break;
    case CAR_STATE_INIT:
    default:
        StateMachine_StopMotionModules();
        break;
    }
}

static void StateMachine_IdleTask(void)
{
}

static void StateMachine_MenuTask(void)
{
}

static void StateMachine_GrayCalibrationTask(void)
{
    Gray_CalibrationSample();
}

/*
 * 作用：检查循迹异常状态，并把无法恢复的异常转成顶层状态。
 * 使用场景：正式循迹和单纯循迹测试共用。
 */
static void StateMachine_CheckTrackingException(void)
{
    TrackingExceptionState exceptionState = TrackingException_GetState();

    if (exceptionState == TRACKING_EXCEPTION_STATE_ADC_FAULT) {
        StateMachine_Enter(CAR_STATE_ERROR);
    } else if (exceptionState == TRACKING_EXCEPTION_STATE_LOST_STOP) {
        StateMachine_Enter(CAR_STATE_STOP);
    }
}

static void StateMachine_TrackingTask(void)
{
    Route_Task();
    Tracking_Task();
    StateMachine_CheckTrackingException();
}

static void StateMachine_TrackingTestTask(void)
{
    Tracking_Task();
    StateMachine_CheckTrackingException();
}

static void StateMachine_MotorTestTask(void)
{
    MotorTest_Task();
}

static void StateMachine_PidMonitorTask(void)
{
}

static void StateMachine_GrayMonitorTask(void)
{
}

static void StateMachine_ExchangeMonitorTask(void)
{
}

static void StateMachine_EncoderMonitorTask(void)
{
}

static void StateMachine_MissionTask(void)
{
}

static void StateMachine_FinishedTask(void)
{
}

static void StateMachine_StopTask(void)
{
}

static void StateMachine_ErrorTask(void)
{
}

void StateMachine_Init(void)
{
    g_carState = CAR_STATE_INIT;
    StateMachine_Enter(CAR_STATE_MENU);
}

void StateMachine_Dispatch(CarEvent event)
{
    if (event == CAR_EVENT_NONE) {
        return;
    }

    LOG_RAW("event: ");
    LOG_LINE(StateMachine_GetEventName(event));

    if (event == CAR_EVENT_ERROR) {
        StateMachine_Enter(CAR_STATE_ERROR);
        return;
    }

    if (event == CAR_EVENT_STOP) {
        StateMachine_Enter(CAR_STATE_STOP);
        return;
    }

    switch (g_carState) {
    case CAR_STATE_IDLE:
    case CAR_STATE_MENU:
        if (event == CAR_EVENT_START) {
            StateMachine_Enter(CAR_STATE_TRACKING);
        } else if (event == CAR_EVENT_GRAY_CALIBRATION_START) {
            StateMachine_Enter(CAR_STATE_GRAY_CALIBRATION);
        } else if (event == CAR_EVENT_MOTOR_TEST_NEXT) {
            StateMachine_Enter(CAR_STATE_MOTOR_TEST);
        } else if (event == CAR_EVENT_TRACKING_TEST_START) {
            StateMachine_Enter(CAR_STATE_TRACKING_TEST);
        } else if (event == CAR_EVENT_PID_MONITOR_START) {
            StateMachine_Enter(CAR_STATE_PID_MONITOR);
        } else if (event == CAR_EVENT_GRAY_MONITOR_START) {
            StateMachine_Enter(CAR_STATE_GRAY_MONITOR);
        } else if (event == CAR_EVENT_EXCHANGE_MONITOR_START) {
            StateMachine_Enter(CAR_STATE_EXCHANGE_MONITOR);
        } else if (event == CAR_EVENT_ENCODER_MONITOR_START) {
            StateMachine_Enter(CAR_STATE_ENCODER_MONITOR);
        } else if (event == CAR_EVENT_MISSION_START) {
            StateMachine_Enter(CAR_STATE_MISSION);
        } else if (event == CAR_EVENT_MENU) {
            StateMachine_Enter(CAR_STATE_MENU);
        }
        break;

    case CAR_STATE_GRAY_CALIBRATION:
        if (event == CAR_EVENT_GRAY_CALIBRATION_SAMPLE) {
            Gray_CalibrationSample();
        } else if (event == CAR_EVENT_GRAY_CALIBRATION_APPLY) {
            Gray_CalibrationApply();
            StateMachine_Enter(CAR_STATE_MENU);
        } else if (event == CAR_EVENT_BACK) {
            StateMachine_Enter(CAR_STATE_MENU);
        }
        break;

    case CAR_STATE_TRACKING:
    case CAR_STATE_TRACKING_TEST:
    case CAR_STATE_PID_MONITOR:
    case CAR_STATE_GRAY_MONITOR:
    case CAR_STATE_EXCHANGE_MONITOR:
    case CAR_STATE_ENCODER_MONITOR:
    case CAR_STATE_MISSION:
        if (event == CAR_EVENT_BACK) {
            StateMachine_Enter(CAR_STATE_MENU);
        } else if (event == CAR_EVENT_TRACKING_DONE) {
            StateMachine_Enter(CAR_STATE_FINISHED);
        }
        break;

    case CAR_STATE_MOTOR_TEST:
        if (event == CAR_EVENT_MOTOR_TEST_NEXT) {
            if (!MotorTest_Next()) {
                StateMachine_Enter(CAR_STATE_MENU);
            }
        } else if (event == CAR_EVENT_BACK) {
            StateMachine_Enter(CAR_STATE_MENU);
        }
        break;

    case CAR_STATE_FINISHED:
    case CAR_STATE_STOP:
        if (event == CAR_EVENT_START) {
            StateMachine_Enter(CAR_STATE_TRACKING);
        } else if ((event == CAR_EVENT_BACK) ||
            (event == CAR_EVENT_CLEAR_ERROR) ||
            (event == CAR_EVENT_MENU)) {
            StateMachine_Enter(CAR_STATE_MENU);
        }
        break;

    case CAR_STATE_ERROR:
        if ((event == CAR_EVENT_CLEAR_ERROR) ||
            (event == CAR_EVENT_BACK)) {
            StateMachine_Enter(CAR_STATE_MENU);
        }
        break;

    case CAR_STATE_INIT:
    default:
        StateMachine_Enter(CAR_STATE_MENU);
        break;
    }
}

void StateMachine_Task(void)
{
    switch (g_carState) {
    case CAR_STATE_IDLE:
        StateMachine_IdleTask();
        break;
    case CAR_STATE_MENU:
        StateMachine_MenuTask();
        break;
    case CAR_STATE_GRAY_CALIBRATION:
        StateMachine_GrayCalibrationTask();
        break;
    case CAR_STATE_TRACKING:
        StateMachine_TrackingTask();
        break;
    case CAR_STATE_TRACKING_TEST:
        StateMachine_TrackingTestTask();
        break;
    case CAR_STATE_MOTOR_TEST:
        StateMachine_MotorTestTask();
        break;
    case CAR_STATE_PID_MONITOR:
        StateMachine_PidMonitorTask();
        break;
    case CAR_STATE_GRAY_MONITOR:
        StateMachine_GrayMonitorTask();
        break;
    case CAR_STATE_EXCHANGE_MONITOR:
        StateMachine_ExchangeMonitorTask();
        break;
    case CAR_STATE_ENCODER_MONITOR:
        StateMachine_EncoderMonitorTask();
        break;
    case CAR_STATE_MISSION:
        StateMachine_MissionTask();
        break;
    case CAR_STATE_FINISHED:
        StateMachine_FinishedTask();
        break;
    case CAR_STATE_STOP:
        StateMachine_StopTask();
        break;
    case CAR_STATE_ERROR:
        StateMachine_ErrorTask();
        break;
    case CAR_STATE_INIT:
    default:
        break;
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
    case CAR_STATE_IDLE:
        return "idle";
    case CAR_STATE_MENU:
        return "menu";
    case CAR_STATE_GRAY_CALIBRATION:
        return "gray_calibration";
    case CAR_STATE_TRACKING:
        return "tracking";
    case CAR_STATE_TRACKING_TEST:
        return "tracking_test";
    case CAR_STATE_MOTOR_TEST:
        return "motor_test";
    case CAR_STATE_PID_MONITOR:
        return "pid_monitor";
    case CAR_STATE_GRAY_MONITOR:
        return "gray_monitor";
    case CAR_STATE_EXCHANGE_MONITOR:
        return "exchange_monitor";
    case CAR_STATE_ENCODER_MONITOR:
        return "encoder_monitor";
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
