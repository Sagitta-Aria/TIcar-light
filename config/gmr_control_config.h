#ifndef GMR_CONTROL_CONFIG_H
#define GMR_CONTROL_CONFIG_H

/* TIMA0 底盘 PWM，Task2 串口百分比按 0..1200 raw count 换算。 */
#define CHASSIS_PWM_PERIOD_COUNTS             (1600U)
#define CHASSIS_PWM_HARDWARE_MAX_COUNTS       (CHASSIS_PWM_PERIOD_COUNTS - 1U)
#define CHASSIS_PWM_LIMIT_COUNTS              (1200U)

/* 未完成实测前关闭零目标反向阻尼，避免手推或停车时突然反冲。 */
#define CHASSIS_ZERO_TARGET_BRAKE_ENABLE       (0U)
#define CHASSIS_ZERO_TARGET_BRAKE_DEADBAND_COUNTS_PER_PERIOD (0U)
#define CHASSIS_ZERO_TARGET_BRAKE_KP_Q1024     (0L)
#define CHASSIS_ZERO_TARGET_BRAKE_PWM_LIMIT_COUNTS (0U)

/* 控制器每10ms运行一次，对外继续使用 count/20ms，兼容 Task5 协议。 */
#define CHASSIS_CONTROL_PERIOD_MS             (10U)
#define CHASSIS_SPEED_UNIT_PERIOD_MS          (20U)
#define CHASSIS_TARGET_LIMIT_CPS              (5000U)
#define CHASSIS_Q1024_SCALE                   (1024L)
#define CHASSIS_CONTROL_HZ \
    (1000U / CHASSIS_CONTROL_PERIOD_MS)
#define CHASSIS_SPEED_UNIT_HZ \
    (1000U / CHASSIS_SPEED_UNIT_PERIOD_MS)
#define CHASSIS_FEEDBACK_COUNT_SCALE \
    (CHASSIS_SPEED_UNIT_PERIOD_MS / CHASSIS_CONTROL_PERIOD_MS)
#define CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD \
    (CHASSIS_TARGET_LIMIT_CPS / CHASSIS_SPEED_UNIT_HZ)

/* 两轮同目标时允许小幅交叉修正，参数可在直行实测后调整。 */
#define CHASSIS_STRAIGHT_SYNC_GAIN_Q1024       (1024L)
#define CHASSIS_STRAIGHT_SYNC_LIMIT_COUNTS_PER_PERIOD (5U)

/* Menu2 Task1 的速度选项。未标定闭环前默认开环20%。 */
#define CHASSIS_DEBUG_SPEED_MIN                (10U)
#define CHASSIS_DEBUG_SPEED_MAX                (100U)
#define CHASSIS_DEBUG_SPEED_STEP               (10U)
#define CHASSIS_TASK1_DEFAULT_CLOSED_LOOP      (0U)
#define CHASSIS_TASK1_CLOSED_SPEED_DEFAULT     (20U)
#define CHASSIS_TASK1_OPEN_SPEED_DEFAULT       (20U)

/* Task6主车每50ms记录实际编码增量，从车收到OVER后按距离闭环回放。 */
#define GMR_BLUETOOTH_MISSION_SAMPLE_PERIOD_MS (50U)
#define GMR_BLUETOOTH_MISSION_MAX_SAMPLES      (2048U)
#define GMR_BLUETOOTH_MISSION_TARGET_TURNS     (4U)
#define GMR_BLUETOOTH_MISSION_READY_PERIOD_MS  (250U)
#define GMR_BLUETOOTH_MISSION_RETRY_PERIOD_MS  (500U)

/* 距离外环把累计编码误差换算成count/20ms速度目标，再交给现有PI。 */
#define GMR_BLUETOOTH_DISTANCE_TOLERANCE_COUNTS (4U)
#define GMR_BLUETOOTH_DISTANCE_KP_Q1024          (512L)
#define GMR_BLUETOOTH_DISTANCE_MIN_SPEED_COUNTS_PER_PERIOD (6U)
#define GMR_BLUETOOTH_DISTANCE_MAX_SPEED_COUNTS_PER_PERIOD (35U)
#define GMR_BLUETOOTH_DISTANCE_SEGMENT_TIMEOUT_MS (2000U)

/*
 * GMR 编码器 PI 初始值：只保证参数合法和输出保守，不代表实车最终值。
 * 先用 Task2 的 pwm/set 测两点，再回写 Start/RunStart/FF；随后调 Kp/Ki。
 */
#define CHASSIS_START_PWM_SPEED_THRESHOLD_COUNTS_PER_PERIOD (5U)   //判断是否速度小于xx启用不同的start前馈
#define CHASSIS_LEFT_START_PWM_COUNTS           (80)
#define CHASSIS_LEFT_RUN_START_PWM_COUNTS       (80)
#define CHASSIS_LEFT_KP_Q1024                (8192L)
#define CHASSIS_LEFT_KI_Q1024                   (1024L)
#define CHASSIS_LEFT_FF_Q1024                   (7600L)
#define CHASSIS_LEFT_INTEGRAL_LIMIT_PWM_COUNTS  (120)
#define CHASSIS_RIGHT_START_PWM_COUNTS          (60)
#define CHASSIS_RIGHT_RUN_START_PWM_COUNTS      (60)
#define CHASSIS_RIGHT_KP_Q1024               (8192L)
#define CHASSIS_RIGHT_KI_Q1024                  (1024L)
#define CHASSIS_RIGHT_FF_Q1024                  (8100)
#define CHASSIS_RIGHT_INTEGRAL_LIMIT_PWM_COUNTS (120)
#define CHASSIS_LEFT_STARTUP_COMPENSATION_PWM_COUNTS (0)  //起步积分补偿

/* 正 PWM 必须得到正反馈；若方向相反只改这里或板级 REVERSE。 */
#define CHASSIS_LEFT_ENCODER_SIGN             (-1)
#define CHASSIS_RIGHT_ENCODER_SIGN             (1)

/*
 * Task5外部M0姿态航向保持。第一帧有效yaw作为上电目标，车体向右偏时
 * error=current-target为正，按left=base-correction、right=base+correction
 * 向左修正。修正量不超过base，保证内轮只减速而不会反转。
 */
#define GMR_M0_YAW_STALE_MS                    (100U)
#define GMR_M0_YAW_COMMAND_MAX_DEGREES         (180)
#define GMR_M0_YAW_DEADBAND_X100              (1000U) /* 正负10.00度内保持基础速度。 */
#define GMR_M0_YAW_KP_Q1024                     (8L)
#define GMR_M0_YAW_BASE_SPEED_COUNTS_PER_PERIOD (20U)
#define GMR_M0_YAW_MIN_CORRECTION_COUNTS_PER_PERIOD (6U)
#define GMR_M0_YAW_MAX_CORRECTION_COUNTS_PER_PERIOD (20U)
#define GMR_M0_YAW_CHASSIS_DIRECTION_SIGN       (1)

#if ((CHASSIS_CONTROL_PERIOD_MS == 0U) || \
    (CHASSIS_SPEED_UNIT_PERIOD_MS % CHASSIS_CONTROL_PERIOD_MS != 0U))
#error "GMR chassis periods must use an integer feedback scale"
#endif

#if ((CHASSIS_LEFT_ENCODER_SIGN != 1) && \
    (CHASSIS_LEFT_ENCODER_SIGN != -1))
#error "GMR left encoder sign must be +1 or -1"
#endif

#if ((CHASSIS_RIGHT_ENCODER_SIGN != 1) && \
    (CHASSIS_RIGHT_ENCODER_SIGN != -1))
#error "GMR right encoder sign must be +1 or -1"
#endif

#if ((GMR_M0_YAW_CHASSIS_DIRECTION_SIGN != 1) && \
    (GMR_M0_YAW_CHASSIS_DIRECTION_SIGN != -1))
#error "GMR M0 yaw chassis direction sign must be +1 or -1"
#endif

#if ((GMR_M0_YAW_MIN_CORRECTION_COUNTS_PER_PERIOD == 0U) || \
    (GMR_M0_YAW_MIN_CORRECTION_COUNTS_PER_PERIOD > \
        GMR_M0_YAW_MAX_CORRECTION_COUNTS_PER_PERIOD) || \
    (GMR_M0_YAW_MAX_CORRECTION_COUNTS_PER_PERIOD > \
        GMR_M0_YAW_BASE_SPEED_COUNTS_PER_PERIOD) || \
    ((GMR_M0_YAW_BASE_SPEED_COUNTS_PER_PERIOD + \
        GMR_M0_YAW_MAX_CORRECTION_COUNTS_PER_PERIOD) > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD))
#error "GMR M0 yaw base/correction speed limits are invalid"
#endif

#if ((GMR_M0_YAW_STALE_MS == 0U) || \
    (GMR_M0_YAW_DEADBAND_X100 > 18000U) || \
    (GMR_M0_YAW_KP_Q1024 < 0L))
#error "GMR M0 yaw stale/deadband/gain configuration is invalid"
#endif

#if ((GMR_BLUETOOTH_DISTANCE_TOLERANCE_COUNTS == 0U) || \
    (GMR_BLUETOOTH_DISTANCE_KP_Q1024 <= 0L) || \
    (GMR_BLUETOOTH_DISTANCE_MIN_SPEED_COUNTS_PER_PERIOD == 0U) || \
    (GMR_BLUETOOTH_DISTANCE_MIN_SPEED_COUNTS_PER_PERIOD > \
        GMR_BLUETOOTH_DISTANCE_MAX_SPEED_COUNTS_PER_PERIOD) || \
    (GMR_BLUETOOTH_DISTANCE_MAX_SPEED_COUNTS_PER_PERIOD > \
        CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD) || \
    (GMR_BLUETOOTH_DISTANCE_SEGMENT_TIMEOUT_MS < \
        CHASSIS_CONTROL_PERIOD_MS))
#error "GMR Bluetooth distance replay configuration is invalid"
#endif

#endif
