#include "app.h"

#include "board.h"
#include "board_config.h"
#include "delay.h"
#include "gimbal.h"
#include "gimbal_motor_test.h"
#include "gimbal_test.h"
#include "key.h"
#include "link.h"
#include "log_uart.h"
#include "menu.h"
#include "motor.h"
#include "motor_no_yaw.h"
#include "motor_track.h"
#include "motor_enable_test.h"
#include "oled.h"
#include "pose_solver.h"
#include "route.h"
#include "state_machine.h"
#include "staticconfig.h"
#include "stepper_pin_test.h"
#include "track_step_test.h"
#include "tracking.h"

/*
 * 作用：把两个实体按键翻译成菜单/状态机事件。
 * 使用场景：App_Task 每轮取到按键事件后调用。
 * 说明：菜单内 K1 切换、K2 确认；测试页 K2 长按退出。
 */
static void App_HandleKeyEvent(KeyEvent event)
{
    CarState state;

    if (event == KEY_EVENT_NONE) {
        return;
    }

    /* 按键测试日志：如果一按出现多行，说明硬件抖动或消抖还要继续加强。 */
    if (event == KEY_EVENT_1) {
        LOG_LINE("key: K1 PB9");
    } else if (event == KEY_EVENT_2) {
        LOG_LINE("key: K2 PB8");
    } else if (event == KEY_EVENT_1_LONG) {
        LOG_LINE("key: K1 long");
    } else if (event == KEY_EVENT_2_LONG) {
        LOG_LINE("key: K2 long");
    }

    state = StateMachine_GetState();
    if (event == KEY_EVENT_1_LONG) {
        return;
    }

    if (event == KEY_EVENT_2_LONG) {
        if (state == CAR_STATE_MENU) {
            (void)Menu_Back();
            return;
        }
        if ((state == CAR_STATE_TRACKING_TEST) ||
            (state == CAR_STATE_MOTOR_TRACK) ||
            (state == CAR_STATE_MOTOR_NO_YAW) ||
            (state == CAR_STATE_MOTOR_GRAY_TEST) ||
            (state == CAR_STATE_GIMBAL_MOTOR_TEST) ||
            (state == CAR_STATE_GIMBAL_TEST) ||
            (state == CAR_STATE_MOTOR_ENABLE_TEST) ||
            (state == CAR_STATE_MISSION)) {
            StateMachine_Dispatch(CAR_EVENT_BACK);
        }
        return;
    }

    if (state == CAR_STATE_MENU) {
        if (event == KEY_EVENT_1) {
            Menu_Next();
        } else if (event == KEY_EVENT_2) {
            StateMachine_Dispatch(Menu_Confirm());
        }
        return;
    }

    if (state == CAR_STATE_GRAY_CALIBRATION) {
        if (event == KEY_EVENT_1) {
            Menu_GrayCalibrationNext();
        } else if (event == KEY_EVENT_2) {
            StateMachine_Dispatch(Menu_GrayCalibrationConfirm());
        }
        return;
    }

    if (state == CAR_STATE_TRACKING_TEST) {
        if (event == KEY_EVENT_1) {
            TrackStepTest_IncreaseSpeed();
            Menu_RequestRefresh();
        } else if (event == KEY_EVENT_2) {
            TrackStepTest_DecreaseSpeed();
            Menu_RequestRefresh();
        }
        return;
    }

    if (state == CAR_STATE_GIMBAL_MOTOR_TEST) {
        if (event == KEY_EVENT_1) {
            GimbalMotorTest_IncreaseSpeed();
            Menu_RequestRefresh();
        } else if (event == KEY_EVENT_2) {
            GimbalMotorTest_DecreaseSpeed();
            Menu_RequestRefresh();
        }
        return;
    }

    if (state == CAR_STATE_ERROR) {
        if (event == KEY_EVENT_1) {
            StateMachine_Dispatch(CAR_EVENT_CLEAR_ERROR);
        } else if (event == KEY_EVENT_2) {
            StateMachine_Dispatch(CAR_EVENT_BACK);
        }
        return;
    }

    if ((state == CAR_STATE_STOP) || (state == CAR_STATE_FINISHED) ||
        (state == CAR_STATE_IDLE)) {
        if (event == KEY_EVENT_1) {
            StateMachine_Dispatch(CAR_EVENT_MENU);
        } else if (event == KEY_EVENT_2) {
            StateMachine_Dispatch(CAR_EVENT_BACK);
        }
        return;
    }

    if (event == KEY_EVENT_2) {
        StateMachine_Dispatch(CAR_EVENT_BACK);
    }
}

/*
 * 作用：初始化所有 app 层模块。
 * 使用场景：Board_Init 完成且没有致命错误后调用一次。
 * 说明：这里允许按顺序初始化模块，但不要放底层引脚配置；硬件初始化属于 system/hardware。
 */
void App_Init(void)
{
#if CAR_GIMBAL_PIN_TEST_BUILD
    StepperPinTest_Start();
    return;
#endif

    Board_ShowBootProgress("I2C OK", "UART OK", "Gray OK", "APP...", "");
    LOG_LINE("app: init begin");
    delay_ms(100U);

    Tracking_Init();
    LOG_LINE("app: tracking init ok");
    TrackStepTest_Init();
    LOG_LINE("app: track step test init ok");
    MotorTrack_Init();
    LOG_LINE("app: motor track init ok");
    MotorNoYaw_Init();
    LOG_LINE("app: motor no yaw init ok");
    Route_Init();
    LOG_LINE("app: route init ok");
    StaticConfig_Init();
    LOG_LINE("app: static config init ok");
    Gimbal_Init();
    LOG_LINE("app: gimbal init ok");
    GimbalTest_Init();
    LOG_LINE("app: gimbal test init ok");
    GimbalMotorTest_Init();
    LOG_LINE("app: gimbal motor test init ok");
    MotorEnableTest_Init();
    LOG_LINE("app: motor enable test init ok");
    PoseSolver_Init();
    LOG_LINE("app: pose solver init ok");
    Menu_Init();
    LOG_LINE("app: menu init ok");
    LOG_LINE("light-car ccs1.2 init ok");
    StateMachine_Init();

    Board_ShowBootProgress("I2C OK", "UART OK", "Gray OK", "APP OK", "");
    delay_ms(200U);

    if (Board_IsOledAvailable() != 0U) {
        OLED_Clear();
    }
    Menu_Task(StateMachine_GetState());
}

/*
 * 作用：应用层周期调度。
 * 使用场景：main while(1) 中反复调用。
 * 说明：中断只做轻量收发，耗时解析和状态更新都放在这里，最后用固定 delay 控制循环节拍。
 */
void App_Task(void)
{
#if CAR_GIMBAL_PIN_TEST_BUILD
    StepperPinTest_Task();
    return;
#endif

    Key_Task();
    App_HandleKeyEvent(Key_PopEvent());

    StateMachine_Task();

    Menu_Task(StateMachine_GetState());
    LogUart_Task();
    Link_Task();
    PoseSolver_Task();
    Gimbal_Task();
    Motor_Task();
    delay_ms(CAR_APP_LOOP_DELAY_MS);
}
