#include "state_machine.h"

#include "gimbal_motor_test.h"
#include "gimbal_test.h"
#include "gray.h"
#include "log_uart.h"
#include "motion.h"
#include "motor_enable_test.h"
#include "route.h"
#include "tracking.h"
#include "tracking_exception.h"

static CarState g_carState = CAR_STATE_INIT;
static uint8_t g_missionId;

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
    case CAR_EVENT_TRACKING_TEST_START:
        return "tracking_test_start";
    case CAR_EVENT_GIMBAL_TEST_START:
        return "gimbal_test_start";
    case CAR_EVENT_GIMBAL_MOTOR_TEST_START:
        return "gimbal_motor_test_start";
    case CAR_EVENT_MOTOR_ENABLE_TEST_START:
        return "motor_enable_test_start";
    case CAR_EVENT_MISSION_1_START:
        return "mission_1_start";
    case CAR_EVENT_MISSION_2_START:
        return "mission_2_start";
    case CAR_EVENT_MISSION_3_START:
        return "mission_3_start";
    case CAR_EVENT_MISSION_4_START:
        return "mission_4_start";
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
 * 作用：把任务菜单事件转换成任务编号。
 * 使用场景：主菜单选择任务 1~4 后，状态机进入同一个任务状态。
 */
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
    default:
        return 0U;
    }
}

/*
 * 作用：停止所有会让车运动的模块。
 * 使用场景：进入菜单、监视、校准、停止、错误等非运行状态时。
 */
static void StateMachine_StopMotionModules(void)
{
    Tracking_SetEnabled(0U);
    GimbalTest_Stop();
    GimbalMotorTest_Stop();
    MotorEnableTest_Stop();
    Route_Stop();
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
    TrackingException_Reset();
    Tracking_SetEnabled(1U);
    LOG_LINE("state: tracking");
}

/*
 * 作用：进入单纯循迹测试状态。
 * 使用场景：菜单里的 Track Test，只跑灰度循迹，不跑路线外环。
 */
static void StateMachine_EnterTrackingTest(void)
{
    Route_Stop();
    TrackingException_Reset();
    Tracking_SetEnabled(1U);
    LOG_LINE("state: tracking test");
}

/*
 * 作用：进入云台测试状态。
 * 使用场景：菜单里的 Gimbal Test，只接收视觉 Link 数据并追踪目标。
 */
static void StateMachine_EnterGimbalTest(void)
{
    Tracking_SetEnabled(0U);
    Route_Stop();
    Motion_Stop();
    GimbalMotorTest_Stop();
    GimbalTest_Start();
    LOG_LINE("state: gimbal test");
}

/*
 * 作用：进入云台电机测试状态。
 * 使用场景：菜单里的 Gimbal Test / Motor Test，上电记零后慢速转 90 度。
 */
static void StateMachine_EnterGimbalMotorTest(void)
{
    Tracking_SetEnabled(0U);
    Route_Stop();
    Motion_Stop();
    GimbalTest_Stop();
    GimbalMotorTest_Start();
    LOG_LINE("state: gimbal motor test");
}

/*
 * 作用：进入四电机使能测试状态。
 * 使用场景：菜单里的 Gimbal Test / Enable Test，只拉 EN，不发 STEP。
 */
static void StateMachine_EnterMotorEnableTest(void)
{
    Tracking_SetEnabled(0U);
    Route_Stop();
    Motion_Stop();
    GimbalTest_Stop();
    GimbalMotorTest_Stop();
    MotorEnableTest_Start();
    LOG_LINE("state: motor enable test");
}

/*
 * 作用：进入赛题任务状态。
 * 使用场景：主菜单里的 Mission，具体赛题流程后续再填。
 */
static void StateMachine_EnterMission(void)
{
    StateMachine_StopMotionModules();
    LOG_RAW("state: mission ");
    LogUart_SendUnsigned(g_missionId);
    LOG_LINE("");
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
 * 使用场景：灰度采样连续异常、后续硬件故障等。
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
    case CAR_STATE_GIMBAL_TEST:
        StateMachine_EnterGimbalTest();
        break;
    case CAR_STATE_GIMBAL_MOTOR_TEST:
        StateMachine_EnterGimbalMotorTest();
        break;
    case CAR_STATE_MOTOR_ENABLE_TEST:
        StateMachine_EnterMotorEnableTest();
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

    if (exceptionState == TRACKING_EXCEPTION_STATE_SENSOR_FAULT) {
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

static void StateMachine_GimbalTestTask(void)
{
    GimbalTest_Task();
}

static void StateMachine_GimbalMotorTestTask(void)
{
    GimbalMotorTest_Task();
}

static void StateMachine_MotorEnableTestTask(void)
{
    MotorEnableTest_Task();
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
    g_missionId = 0U;
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
        {
            uint8_t missionId = StateMachine_GetMissionIdFromEvent(event);

            if (missionId != 0U) {
                g_missionId = missionId;
                StateMachine_Enter(CAR_STATE_MISSION);
                break;
            }
        }

        if (event == CAR_EVENT_START) {
            StateMachine_Enter(CAR_STATE_TRACKING);
        } else if (event == CAR_EVENT_GRAY_CALIBRATION_START) {
            StateMachine_Enter(CAR_STATE_GRAY_CALIBRATION);
        } else if (event == CAR_EVENT_TRACKING_TEST_START) {
            StateMachine_Enter(CAR_STATE_TRACKING_TEST);
        } else if (event == CAR_EVENT_GIMBAL_TEST_START) {
            StateMachine_Enter(CAR_STATE_GIMBAL_TEST);
        } else if (event == CAR_EVENT_GIMBAL_MOTOR_TEST_START) {
            StateMachine_Enter(CAR_STATE_GIMBAL_MOTOR_TEST);
        } else if (event == CAR_EVENT_MOTOR_ENABLE_TEST_START) {
            StateMachine_Enter(CAR_STATE_MOTOR_ENABLE_TEST);
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
    case CAR_STATE_GIMBAL_TEST:
    case CAR_STATE_GIMBAL_MOTOR_TEST:
    case CAR_STATE_MOTOR_ENABLE_TEST:
    case CAR_STATE_MISSION:
        if (event == CAR_EVENT_BACK) {
            StateMachine_Enter(CAR_STATE_MENU);
        } else if (event == CAR_EVENT_TRACKING_DONE) {
            StateMachine_Enter(CAR_STATE_FINISHED);
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
    case CAR_STATE_GIMBAL_TEST:
        StateMachine_GimbalTestTask();
        break;
    case CAR_STATE_GIMBAL_MOTOR_TEST:
        StateMachine_GimbalMotorTestTask();
        break;
    case CAR_STATE_MOTOR_ENABLE_TEST:
        StateMachine_MotorEnableTestTask();
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
    case CAR_STATE_GIMBAL_TEST:
        return "gimbal_test";
    case CAR_STATE_GIMBAL_MOTOR_TEST:
        return "gimbal_motor_test";
    case CAR_STATE_MOTOR_ENABLE_TEST:
        return "motor_enable_test";
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
