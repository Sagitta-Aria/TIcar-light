#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* 共享车体 yaw 估计器：Q1024 系数可在 Task5 中在线调整。 */
#define BODY_MOTION_PERIOD_MS                    (10U)
#define BODY_MOTION_CALIBRATION_SAMPLES          (100U)
#define BODY_MOTION_GYRO_ALPHA_Q1024             (256U)
#define BODY_MOTION_YAW_BETA_Q1024               (64U)
#define BODY_MOTION_PREDICTION_MS                 (0U)
#define BODY_MOTION_SENSOR_STALE_MS               (100U)

/* Task8 云台 yaw 姿态保持默认参数。 */
#define GIMBAL_ATTITUDE_STEPS_PER_REVOLUTION      (3200U)
#define GIMBAL_ATTITUDE_DIRECTION_SIGN            (-1)
#define GIMBAL_ATTITUDE_KFF_Q1024                 (1024U)
#define GIMBAL_ATTITUDE_KP_Q1024                  (1024U)
#define GIMBAL_ATTITUDE_MAX_SPEED_SPS             (1000U)
#define GIMBAL_ATTITUDE_ACCEL_STEP_SPS            (3U)
#define GIMBAL_ATTITUDE_POSITION_LIMIT_STEPS       (1600U)

/* TIMA0 底盘 PWM：32 MHz / 1600 = 20 kHz，全部 PWM 数值均为定时器 count。 */
#define CHASSIS_PWM_PERIOD_COUNTS             (1600U)
#define CHASSIS_PWM_HARDWARE_MAX_COUNTS       (CHASSIS_PWM_PERIOD_COUNTS - 1U)
#define CHASSIS_PWM_LIMIT_COUNTS              (1200U)  /* 软件输出限幅；100% 标定命令对应此值。 */

/*
 * 编码底盘速度环每 20 ms 运行一次，内部目标单位为 encoder count/控制周期。
 * 旧 CPS 接口仅保留作兼容边界，进入速度环前只换算一次。
 */
#define CHASSIS_CONTROL_PERIOD_MS             (20U)
#define CHASSIS_TARGET_LIMIT_CPS              (5000U)
#define CHASSIS_Q1024_SCALE                   (1024L)

#if ((CHASSIS_CONTROL_PERIOD_MS == 0U) || \
    ((1000U % CHASSIS_CONTROL_PERIOD_MS) != 0U))
#error "CHASSIS_CONTROL_PERIOD_MS must divide 1000 exactly"
#endif

#define CHASSIS_CONTROL_HZ \
    (1000U / CHASSIS_CONTROL_PERIOD_MS)

#if ((CHASSIS_TARGET_LIMIT_CPS % CHASSIS_CONTROL_HZ) != 0U)
#error "CHASSIS_TARGET_LIMIT_CPS must map to an integer count/period"
#endif

/* 与 Task5 target/move 相同的速度刻度；当前范围为 -100..100 count/20ms。 */
#define CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD \
    (CHASSIS_TARGET_LIMIT_CPS / CHASSIS_CONTROL_HZ)

/*
 * 直线同步：正常闭环且左右原始目标相同时，用本周期左右编码器速度差
 * 对两侧目标做等量反向修正。1024表示完整跟随两轮平均速度，0表示关闭。
 * LIMIT限制单侧每个20ms周期最多增减的encoder count，避免低速修正过猛。
 */
#define CHASSIS_STRAIGHT_SYNC_GAIN_Q1024       (1024L)
#define CHASSIS_STRAIGHT_SYNC_LIMIT_COUNTS_PER_PERIOD (5U)

#if (CHASSIS_STRAIGHT_SYNC_GAIN_Q1024 < 0L)
#error "CHASSIS_STRAIGHT_SYNC_GAIN_Q1024 must be non-negative"
#endif

#if (CHASSIS_STRAIGHT_SYNC_LIMIT_COUNTS_PER_PERIOD > \
    CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD)
#error "CHASSIS_STRAIGHT_SYNC_LIMIT exceeds the chassis target limit"
#endif

/*
 * Task6 调试页速度范围。闭环单位为 count/20ms，开环单位为 PWM%。
 * 两种模式共用 10~100 的菜单刻度，K1 每次增加 10，到上限后回到 10。
 */
#define CHASSIS_DEBUG_SPEED_MIN                (10U)
#define CHASSIS_DEBUG_SPEED_MAX                (100U)
#define CHASSIS_DEBUG_SPEED_STEP               (10U)

#define CHASSIS_TASK6_DEFAULT_CLOSED_LOOP      (1U)
#define CHASSIS_TASK6_CLOSED_SPEED_DEFAULT     (40U)
#define CHASSIS_TASK6_OPEN_SPEED_DEFAULT       (40U)

#if ((CHASSIS_DEBUG_SPEED_MIN == 0U) || \
    (CHASSIS_DEBUG_SPEED_MIN > CHASSIS_DEBUG_SPEED_MAX) || \
    (CHASSIS_DEBUG_SPEED_STEP == 0U) || \
    (((CHASSIS_DEBUG_SPEED_MAX - CHASSIS_DEBUG_SPEED_MIN) % \
        CHASSIS_DEBUG_SPEED_STEP) != 0U))
#error "CHASSIS_DEBUG_SPEED range/step is invalid"
#endif

#if ((CHASSIS_DEBUG_SPEED_MAX > 100U) || \
    (CHASSIS_DEBUG_SPEED_MAX > CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD))
#error "CHASSIS_DEBUG_SPEED_MAX exceeds open/closed-loop limit"
#endif

#if (CHASSIS_TASK6_DEFAULT_CLOSED_LOOP > 1U)
#error "Task6 default mode must be 0(open) or 1(closed)"
#endif

#if ((CHASSIS_TASK6_CLOSED_SPEED_DEFAULT < CHASSIS_DEBUG_SPEED_MIN) || \
    (CHASSIS_TASK6_CLOSED_SPEED_DEFAULT > CHASSIS_DEBUG_SPEED_MAX) || \
    (CHASSIS_TASK6_OPEN_SPEED_DEFAULT < CHASSIS_DEBUG_SPEED_MIN) || \
    (CHASSIS_TASK6_OPEN_SPEED_DEFAULT > CHASSIS_DEBUG_SPEED_MAX))
#error "Task6 default speed is outside debug menu range"
#endif

/*
 * 左右轮参数必须分别架空标定，在线修改只保存在 RAM，复位后恢复这里的值。
 * START/RUN_START/INTEGRAL_LIMIT 单位为 PWM count；FF/KP/KI 使用 Q1024 定点数。
 * 实际速度绝对值小于阈值时使用START，否则使用RUN_START。
 * FF 表示每个 20 ms 编码器 count 对应的 PWM count，再乘 1024 保存。
 */
#define CHASSIS_START_PWM_SPEED_THRESHOLD_COUNTS_PER_PERIOD (15U)
#define CHASSIS_LEFT_START_PWM_COUNTS         (300)
#define CHASSIS_LEFT_RUN_START_PWM_COUNTS     (198)
#define CHASSIS_LEFT_KP_Q1024                 (2048L)
#define CHASSIS_LEFT_KI_Q1024                 (0L)
#define CHASSIS_LEFT_FF_Q1024                 (7557L)
#define CHASSIS_LEFT_INTEGRAL_LIMIT_PWM_COUNTS (120)
#define CHASSIS_RIGHT_START_PWM_COUNTS        (260)
#define CHASSIS_RIGHT_RUN_START_PWM_COUNTS    (148)
#define CHASSIS_RIGHT_KP_Q1024                (2048L)
#define CHASSIS_RIGHT_KI_Q1024                (0L)
#define CHASSIS_RIGHT_FF_Q1024                (6200L)
#define CHASSIS_RIGHT_INTEGRAL_LIMIT_PWM_COUNTS (120)

#if (CHASSIS_START_PWM_SPEED_THRESHOLD_COUNTS_PER_PERIOD > \
    CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD)
#error "CHASSIS_START_PWM_SPEED_THRESHOLD exceeds the target limit"
#endif

/*
 * 整车同向起步时预装到积分项的左右差速补偿，单位为 raw PWM count。
 * 正数：增强左轮并等量减弱右轮；负数：增强右轮并等量减弱左轮。
 * 保持为有符号 int；预装后由正常积分误差连续调整，不按时间取消。
 */
#define CHASSIS_LEFT_STARTUP_COMPENSATION_PWM_COUNTS (0)  //预装载积分

/* 编码器正方向修正：1 保持计数方向，-1 反转计数方向。 */
#define CHASSIS_LEFT_ENCODER_SIGN             (1)
#define CHASSIS_RIGHT_ENCODER_SIGN            (-1)

#if ((CHASSIS_LEFT_ENCODER_SIGN != 1) && \
    (CHASSIS_LEFT_ENCODER_SIGN != -1))
#error "CHASSIS_LEFT_ENCODER_SIGN must be 1 or -1"
#endif

#if ((CHASSIS_RIGHT_ENCODER_SIGN != 1) && \
    (CHASSIS_RIGHT_ENCODER_SIGN != -1))
#error "CHASSIS_RIGHT_ENCODER_SIGN must be 1 or -1"
#endif

#endif
