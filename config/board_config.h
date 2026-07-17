#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

/*
 * CAR_RECOVERY_SAFE_BUILD：芯片恢复用安全构建开关。
 * 1：只跑 PA14 心跳和恢复串口；0：正常比赛固件。
 */
#define CAR_RECOVERY_SAFE_BUILD        (0U)

/*
 * 日志策略：
 * - 启动初始化阶段允许少量日志。
 * - 主循环、按键、菜单、视觉坏帧、NO YAW 运行过程不打印。
 */
#define CAR_ENABLE_LOG_UART            (1U)

/* PA14 状态灯：慢闪表示主循环存活，常亮表示致命错误。 */
#define CAR_ENABLE_PA14_DEBUG_LED      (1U)

/*
 * 两路云台步进轴公共参数。
 * 速度单位为 step/s；TIMG0 每 20us 调度一次 STEP，方向由 DIR 引脚单独控制。
 */
#define CAR_STEPPER_SPEED_MAX_SPS      (10000U) /* yaw/pitch 目标速度绝对值上限。 */
#define CAR_STEPPER_PULSE_HIGH_TICKS   (1U)     /* STEP 高电平保持 1 个 TIMG0 tick，即 20us。 */
#define CAR_STEPPER_RAMP_PERIOD_MS     (1U)     /* 每 1ms 更新一次当前 STEP 速度。 */
#define CAR_STEPPER_ACCEL_STEP_SPS     (500U)   /* 每次斜坡更新最多增加 500 step/s。 */
#define CAR_STEPPER_DECEL_STEP_SPS     (500U)   /* 每次斜坡更新最多减少 500 step/s。 */

/* 云台 EN 无可用 MCU 引脚；该值只记录状态机默认请求，硬件必须固定有效。 */
#define CAR_GIMBAL_ENABLE_DEFAULT_ON   (0U)     /* 0：上电默认不启用云台闭环；1：默认请求启用。 */

/* 底盘左右轮安装方向修正。 */
#define CAR_CHASSIS_LEFT_REVERSE       (1U)
#define CAR_CHASSIS_RIGHT_REVERSE      (0U)

/* K1/K2 按键：低电平按下，正式版不保留按键诊断日志。 */
#define CAR_KEY_ACTIVE_LOW             (1U)

/*
 * 云台闭环参数。
 * pitch 没有回零开关，软限位以每次启用闭环时的位置作为相对 0 度。
 */
#define CAR_GIMBAL_YAW_REVERSE         (1U)  /* 1：反转 yaw 控制方向；0：保持计算方向。 */
#define CAR_GIMBAL_PITCH_REVERSE       (0U)  /* 1：反转 pitch 控制方向；0：保持计算方向。 */
#define CAR_GIMBAL_VISION_TIMEOUT_TICKS (50U) /* 视觉数据 50ms 未更新时停止视觉追点。 */
#define CAR_GIMBAL_PITCH_LIMIT_STEPS   (400U) /* pitch 相对启用位置允许正负 400 STEP。 */

/*
 * Motor NO YAW：Task1 编码底盘循迹参数。
 * 注意：快采样、S5/S6/S7 右直角窗口和强转时间不要随手改。
 * 速度单位与 Task5 target/move 相同：encoder count/20ms，范围 -100..100。
 * 正负号直接对应串口 target；当前实车 Task1 前进方向使用负目标。
 * 距离仍为累计 encoder count。
 */
#define CAR_MOTOR_NO_YAW_BASE_SPEED_COUNTS_PER_PERIOD        (-25)  /* 等同串口 target -25 -25。 */
#define CAR_MOTOR_NO_YAW_MIN_LINE_SPEED_COUNTS_PER_PERIOD    (-40)  /* 差速目标数值下限。 */
#define CAR_MOTOR_NO_YAW_MAX_LINE_SPEED_COUNTS_PER_PERIOD    (0)  /* 差速目标数值上限。 */
#define CAR_MOTOR_NO_YAW_LINE_DEADBAND         (0)     /* 误差死区：灰度误差小于该值时按居中处理。 */
#define CAR_MOTOR_NO_YAW_TURN_GAIN             (5)     /* 修正量 = 灰度误差 * gain / 100。 */
#define CAR_MOTOR_NO_YAW_TURN_LIMIT_COUNTS_PER_PERIOD       (40U)  /* 普通循迹最大差速修正。 */
#define CAR_MOTOR_NO_YAW_TURN_SPEED_COUNTS_PER_PERIOD        (-25)  /* 右直角强转时左轮目标。 */
#define CAR_MOTOR_NO_YAW_TURN_NEAR_SPEED_COUNTS_PER_PERIOD   (-20)  /* 接近粗略参考角时左轮目标。 */
#define CAR_MOTOR_NO_YAW_TURN_MIN_ENCODER_GAP_COUNTS (708U) /* 两次强转之间的最小平均编码距离。 */
#define CAR_MOTOR_NO_YAW_DEFAULT_SEARCH_SPEED_COUNTS_PER_PERIOD (-25) /* 无上一拍命令时的左轮搜线目标。 */
#define CAR_MOTOR_NO_YAW_RIGHT_TURN_WINDOW_MS  (50U)   /* 直角窗口：S5/S6/S7 在该时间窗内都触发才判右直角。 */
#define CAR_MOTOR_NO_YAW_TIMER_SAMPLE_US       (100U)   /* 快采样周期：TIMG0 中断读取 Gray_ReadDigitalMaskFast()。 */
#define CAR_MOTOR_NO_YAW_LINE_CONFIRM_SAMPLES  (2U)     /* 普通循迹mask连续出现2次才更新，约200us。 */
#define CAR_MOTOR_NO_YAW_TURN_REARM_MS         (20U)    /* 出弯后 S5/S6/S7 全部释放 20ms 才允许计下一个右转。 */
#define CAR_MOTOR_NO_YAW_RETURN_CONFIRM_SAMPLES (2U)   /* S4回线连续有效2次，约200us。 */
#define CAR_MOTOR_NO_YAW_TURN_APPROACH_MS      (0U)    /* 触发右直角后继续前进40ms。 */
#define CAR_MOTOR_NO_YAW_TURN_EXIT_OUTER_SPEED_COUNTS_PER_PERIOD (-25) /* 出弯左轮目标，绝对速度25。 */
#define CAR_MOTOR_NO_YAW_TURN_EXIT_INNER_SPEED_COUNTS_PER_PERIOD (-30) /* 出弯右轮目标，绝对速度25。 */
#define CAR_MOTOR_NO_YAW_TURN_EXIT_MS           (40U)   /* 出弯低速前进保持时间。 */
#define CAR_MOTOR_NO_YAW_TURN_HOLD_MS          (5000U)  /* 强转超时：超过该时间仍未找到S4回线则停车。 */
#define CAR_MOTOR_NO_YAW_LINE_LOST_TIMEOUT_MS  (0U)     /* 0=不因丢线停车，继续按搜线目标运行。 */

/* JY61P 只辅助右转减速，最终出弯由灰度S4回线决定；角度单位为0.01度。 */
#define CAR_MOTOR_NO_YAW_TURN_TARGET_ANGLE_X100 (7500U)

/*
 * Task4 编码底盘 NO YAW：独立于 Task1 的循迹参数。
 * 为保持现有任务四参数不变，这组速度仍使用 CPS；装入 NO YAW 配置表时
 * 会校验并换算成 count/20ms。距离仍为 encoder count。
 */
#define CAR_MOTOR_NO_YAW_TASK4_BASE_SPEED_CPS        (1000U)  /* Task4 巡航目标速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_MIN_LINE_SPEED_CPS    (500U)   /* Task4 最低单轮目标速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_MAX_LINE_SPEED_CPS    (5000U)  /* Task4 单轮目标速度上限。 */
#define CAR_MOTOR_NO_YAW_TASK4_LINE_DEADBAND         (30)      /* Task4 灰度误差死区。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_GAIN             (100)     /* Task4 差速修正强度。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_LIMIT_CPS        (800U)    /* Task4 最大差速修正量。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_SPEED_CPS        (2800U)   /* Task4 右直角强转外轮速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_NEAR_SPEED_CPS   (1250U)   /* Task4 接近粗略参考角时外轮速度，25 count/20ms。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_MIN_ENCODER_GAP_COUNTS (10000U) /* Task4 两次强转的最小平均编码距离。 */
#define CAR_MOTOR_NO_YAW_TASK4_DEFAULT_SEARCH_SPEED_CPS (800U) /* Task4 丢线搜线速度。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_APPROACH_MS      (120U)    /* Task4 进弯等待时间。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_OUTER_SPEED_CPS (1250U) /* Task4 出弯左轮，25 count/20ms。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_INNER_SPEED_CPS (1350U) /* Task4 出弯右轮，27 count/20ms。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_EXIT_MS          (80U)     /* Task4 出弯保持时间。 */
#define CAR_MOTOR_NO_YAW_TASK4_TURN_HOLD_MS          (5000U)   /* Task4 强转超时。 */
#define CAR_MOTOR_NO_YAW_TASK4_LINE_LOST_TIMEOUT_MS  (1000U)   /* Task4 丢线停车超时。 */

/*
 * Task4 云台 yaw 随动参数。
 * NEAR/MID/FAR 三组会按 Task4 当前位置切换：近/中/远/中循环。
 * STEPS：每次强转开始后，yaw 额外跟随的 STEP 数，方向由速度符号决定。
 * RATIO_X1000：yaw 基础速度 = 当前底盘平均速度 * ratio / 1000；负数可反向。
 * SPEED_SPS：强转期间单独使用的 yaw 目标速度。
 * ACCEL/DECEL_STEP_SPS：强转期间只给 yaw 轴使用的更快斜坡；DECEL 是降速斜坡。
 */
#define CAR_MISSION4_GIMBAL_NEAR_YAW_STEPS          (3000U)
#define CAR_MISSION4_GIMBAL_NEAR_YAW_RATIO_X1000    (300)
#define CAR_MISSION4_GIMBAL_NEAR_YAW_SPEED_SPS      (6000U)
#define CAR_MISSION4_GIMBAL_NEAR_YAW_ACCEL_STEP_SPS (3000U)  //加速度
#define CAR_MISSION4_GIMBAL_NEAR_YAW_DECEL_STEP_SPS (3000U)

#define CAR_MISSION4_GIMBAL_MID_YAW_STEPS           (3000U)
#define CAR_MISSION4_GIMBAL_MID_YAW_RATIO_X1000     (150)
#define CAR_MISSION4_GIMBAL_MID_YAW_SPEED_SPS       (6000U)
#define CAR_MISSION4_GIMBAL_MID_YAW_ACCEL_STEP_SPS  (3000U)
#define CAR_MISSION4_GIMBAL_MID_YAW_DECEL_STEP_SPS  (3000U)

#define CAR_MISSION4_GIMBAL_FAR_YAW_STEPS           (3000U)
#define CAR_MISSION4_GIMBAL_FAR_YAW_RATIO_X1000     (300)
#define CAR_MISSION4_GIMBAL_FAR_YAW_SPEED_SPS       (6000U)
#define CAR_MISSION4_GIMBAL_FAR_YAW_ACCEL_STEP_SPS  (3000U)
#define CAR_MISSION4_GIMBAL_FAR_YAW_DECEL_STEP_SPS  (3000U)

/* 主循环和 OLED 菜单刷新节拍。 */
#define CAR_APP_LOOP_DELAY_MS          (1U)
#define CAR_MENU_REFRESH_MS            (100U)
#define CAR_MENU_REFRESH_TICKS \
    ((CAR_MENU_REFRESH_MS + CAR_APP_LOOP_DELAY_MS - 1U) / \
        CAR_APP_LOOP_DELAY_MS)

/*
 * 灰度输入。
 * 数字量模式下 Gray_ReadDigitalMaskFast 直接读 GPIO，NO YAW 定时器中断使用这条快路径。
 */
#define CAR_GRAY_INPUT_DIGITAL         (1U)
#define GRAY_DIGITAL_ACTIVE_HIGH       (1U)
#define GRAY_DIGITAL_INPUT_PULL_UP     (1U)
#define GRAY_SENSOR_COUNT              (7U)
#define CAR_GRAY_TRACK_SENSOR_MASK     (0x7FU)

/* 模拟模式保留编译能力，正式车当前不走 ADC 灰度。 */
#define GRAY_ACTIVE_HIGH               (1U)
#define GRAY_ADC_MAX_VALUE             (4095U)
#define GRAY_DEFAULT_THRESHOLD         (2000U)
#define GRAY_ADC_TIMEOUT_COUNT         (100000U)
#define GRAY_FILTER_SAMPLE_COUNT       (1U)
#define GRAY_DIGITAL_CONFIRM_COUNT     (1U)
#define GRAY_LINE_ERROR_SCALE          (100)

#endif
