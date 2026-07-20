#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* ---------- Task4/Task8 固定控制周期 ---------- */
#define BODY_MOTION_PERIOD_MS                       (10U)

#if (BODY_MOTION_PERIOD_MS == 0U)
#error "BODY_MOTION_PERIOD_MS must be greater than zero"
#endif

/*
 * ---------- 视觉目标长度 -> yaw增益线性拟合 ----------
 * K230第5字段是目标表观长度并保留一位小数，30.0~140.0对应300~1400。
 * 超出范围先钳位，再在K=0.8~1.6间线性插值；Q1024避免M0+使用浮点。
 * 端点来自旧三档yaw上限：远档400/中档500=0.8，近档800/中档500=1.6。
 */
#define VISION_TARGET_LENGTH_MIN_X10                  (300)
#define VISION_TARGET_LENGTH_MAX_X10                 (1400)
#define GIMBAL_VISION_YAW_GAIN_Q1024_SCALE           (1024U)
#define GIMBAL_VISION_YAW_GAIN_MIN_Q1024              (819U) /* 约0.8。 */
#define GIMBAL_VISION_YAW_GAIN_MAX_Q1024             (1638U) /* 约1.6。 */

#if (VISION_TARGET_LENGTH_MAX_X10 <= VISION_TARGET_LENGTH_MIN_X10)
#error "VISION target length max must be greater than min"
#endif

#if ((GIMBAL_VISION_YAW_GAIN_MIN_Q1024 == 0U) || \
    (GIMBAL_VISION_YAW_GAIN_MAX_Q1024 < \
        GIMBAL_VISION_YAW_GAIN_MIN_Q1024))
#error "GIMBAL vision yaw gain range is invalid"
#endif

/* ---------- 板载JY61：底座姿态估计与角速度前馈 ---------- */
#define BODY_MOTION_CALIBRATION_SAMPLES             (100U)  /* 静止零偏样本数。 */
#define BODY_MOTION_GYRO_ALPHA_Q1024                (512U)  /* JY61角速度低通；越大响应越快。 */
#define BODY_MOTION_YAW_BETA_Q1024                  (512U)  /* JY61角度校正；越大响应越快。 */
#define BODY_MOTION_PREDICTION_MS                    (10U)  /* JY61姿态预测时间。 */
#define BODY_MOTION_SENSOR_STALE_MS                 (100U)  /* JY61帧超时；超时后只关闭前馈。 */
#define GIMBAL_ATTITUDE_JY61_FEEDFORWARD_SIGN         (1)   /* JY61底座yaw角速度坐标方向。 */
#define GIMBAL_ATTITUDE_JY61_KFF_Q1024              (1024U) /* JY61底座角速度前馈增益，1.0。 */

/*
 * 旧版JY61单传感器姿态环的STEP位置误差Kp调试记录。
 * 当前双传感器Task8中JY61只做前馈，此宏不参与控制计算。
 */
#define GIMBAL_ATTITUDE_JY61_LEGACY_STEP_KP_Q1024  (16384U)

/* ---------- H7：云台yaw角度与角速度反馈 ---------- */
#define H7_GYRO_FEEDBACK_STALE_MS                   (100U)  /* H7任一姿态帧超时后立即停yaw。 */
#define GIMBAL_ATTITUDE_H7_FEEDBACK_SIGN              (-1)   /* H7 yaw角度和角速度坐标方向。 */
#define GIMBAL_ATTITUDE_H7_ANGLE_DEADBAND_X100        (100L) /* 连续角度死区；30表示0.30度。 */
#define GIMBAL_ATTITUDE_H7_ANGLE_KP_Q1024          (8192U) /* H7 yaw角度外环Kp；8.0。 */
#define GIMBAL_ATTITUDE_H7_RATE_KP_Q1024               (100U) /* H7 yaw角速度反馈Kp；0表示暂时关闭。 */

/* ---------- yaw步进执行器：H7反馈与JY61前馈共同使用 ---------- */
#define GIMBAL_ATTITUDE_STEPS_PER_REVOLUTION         (3200U) /* yaw机械旋转一圈所需STEP数。 */
#define GIMBAL_ATTITUDE_MOTOR_DIRECTION_SIGN            (-1) /* 正yaw速度到STEP/DIR的方向映射。 */
#define GIMBAL_ATTITUDE_MAX_SPEED_SPS                (1600U) /* Task4/Task8最终yaw速度限幅。 */
#define GIMBAL_ATTITUDE_ACCEL_STEP_SPS                (200U) /* 每1ms最多增加/减少的SPS。 */
#define GIMBAL_ATTITUDE_POSITION_LIMIT_STEPS          (25600U) /* 相对姿态辅助启动位置的软件行程。 */

/* TIMA0 底盘 PWM：32 MHz / 1600 = 20 kHz，全部 PWM 数值均为定时器 count。 */
#define CHASSIS_PWM_PERIOD_COUNTS             (1600U)
#define CHASSIS_PWM_HARDWARE_MAX_COUNTS       (CHASSIS_PWM_PERIOD_COUNTS - 1U)
#define CHASSIS_PWM_LIMIT_COUNTS              (1200U)  /* 软件输出限幅；100% 标定命令对应此值。 */

/*
 * 零目标速度阻尼：调用方显式允许后，目标为0但编码器仍有速度时，输出
 * 与反馈方向相反的PWM。该分支不使用START、FF或积分，只做比例阻尼并
 * 受独立限幅保护。当前NO YAW强转使用-1内轮目标，不启用此零目标分支。
 */
#define CHASSIS_ZERO_TARGET_BRAKE_ENABLE       (1U)
#define CHASSIS_ZERO_TARGET_BRAKE_DEADBAND_COUNTS_PER_PERIOD (0U)
#define CHASSIS_ZERO_TARGET_BRAKE_KP_Q1024     (2048L)
#define CHASSIS_ZERO_TARGET_BRAKE_PWM_LIMIT_COUNTS (300U)

#if (CHASSIS_ZERO_TARGET_BRAKE_ENABLE > 1U)
#error "CHASSIS_ZERO_TARGET_BRAKE_ENABLE must be 0 or 1"
#endif

#if (CHASSIS_ZERO_TARGET_BRAKE_KP_Q1024 < 0L)
#error "CHASSIS_ZERO_TARGET_BRAKE_KP_Q1024 must be non-negative"
#endif

#if (CHASSIS_ZERO_TARGET_BRAKE_PWM_LIMIT_COUNTS > CHASSIS_PWM_LIMIT_COUNTS)
#error "CHASSIS_ZERO_TARGET_BRAKE_PWM_LIMIT exceeds chassis PWM limit"
#endif

/*
 * 底盘速度环每10 ms运行一次，提高灰度循迹和PWM交叉同步的响应频率。
 * 为避免提频后所有已标定速度翻倍，命令、反馈和Task5仍统一使用
 * encoder count/20ms；底层会把10 ms原始编码器窗口归一化到该速度刻度。
 * 旧CPS接口仅保留作兼容边界，进入速度环前只换算一次。
 */
#define CHASSIS_CONTROL_PERIOD_MS             (10U)
#define CHASSIS_SPEED_UNIT_PERIOD_MS          (20U)
#define CHASSIS_TARGET_LIMIT_CPS              (5000U)
#define CHASSIS_Q1024_SCALE                   (1024L)

#if ((CHASSIS_CONTROL_PERIOD_MS == 0U) || \
    ((1000U % CHASSIS_CONTROL_PERIOD_MS) != 0U))
#error "CHASSIS_CONTROL_PERIOD_MS must divide 1000 exactly"
#endif

#if ((CHASSIS_SPEED_UNIT_PERIOD_MS == 0U) || \
    ((1000U % CHASSIS_SPEED_UNIT_PERIOD_MS) != 0U) || \
    ((CHASSIS_SPEED_UNIT_PERIOD_MS % CHASSIS_CONTROL_PERIOD_MS) != 0U))
#error "CHASSIS speed unit must divide 1000 and contain whole control periods"
#endif

#define CHASSIS_CONTROL_HZ \
    (1000U / CHASSIS_CONTROL_PERIOD_MS)
#define CHASSIS_SPEED_UNIT_HZ \
    (1000U / CHASSIS_SPEED_UNIT_PERIOD_MS)
#define CHASSIS_FEEDBACK_COUNT_SCALE \
    (CHASSIS_SPEED_UNIT_PERIOD_MS / CHASSIS_CONTROL_PERIOD_MS)

#if ((CHASSIS_TARGET_LIMIT_CPS % CHASSIS_SPEED_UNIT_HZ) != 0U)
#error "CHASSIS_TARGET_LIMIT_CPS must map to an integer speed-unit count"
#endif

/* 与Task5 target/move相同的固定速度刻度：-100..100 count/20ms。 */
#define CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD \
    (CHASSIS_TARGET_LIMIT_CPS / CHASSIS_SPEED_UNIT_HZ)

#if (CHASSIS_ZERO_TARGET_BRAKE_DEADBAND_COUNTS_PER_PERIOD > \
    CHASSIS_TARGET_LIMIT_COUNTS_PER_PERIOD)
#error "CHASSIS_ZERO_TARGET_BRAKE_DEADBAND exceeds target limit"
#endif

/*
 * 直线同步：正常闭环且左右原始目标相同时，用本周期左右编码器速度差
 * 对两侧目标做等量反向修正。1024表示完整跟随两轮平均速度，0表示关闭。
 * LIMIT以count/20ms限制单侧修正，避免低速修正过猛。
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
 * 这些是电机和轮组本身的内层速度环参数，Task1/Task4四套外层循迹配置共用。
 * START/RUN_START/INTEGRAL_LIMIT 单位为 PWM count；FF/KP/KI 使用 Q1024 定点数。
 * 从停车或反向起步时使用START；反馈首次达到阈值后单向切到RUN_START。
 * 本次运行中反馈再次跌破阈值不会切回START，避免阈值附近基础PWM跳变。
 * FF表示每个count/20ms对应的PWM count，再乘1024保存。KI仍按20 ms
 * 速度刻度标定，控制器在10 ms执行时按周期比例缩放每次积分增量。
 */
#define CHASSIS_START_PWM_SPEED_THRESHOLD_COUNTS_PER_PERIOD (15U)
#define CHASSIS_LEFT_START_PWM_COUNTS         (200)
#define CHASSIS_LEFT_RUN_START_PWM_COUNTS     (180)
#define CHASSIS_LEFT_KP_Q1024                 (16000L)
#define CHASSIS_LEFT_KI_Q1024                 (100L)
#define CHASSIS_LEFT_FF_Q1024                 (12000L)
#define CHASSIS_LEFT_INTEGRAL_LIMIT_PWM_COUNTS (360)
#define CHASSIS_RIGHT_START_PWM_COUNTS        (160)
#define CHASSIS_RIGHT_RUN_START_PWM_COUNTS    (140)
#define CHASSIS_RIGHT_KP_Q1024                (6200L)
#define CHASSIS_RIGHT_KI_Q1024                (100L)
#define CHASSIS_RIGHT_FF_Q1024                (4096L)
#define CHASSIS_RIGHT_INTEGRAL_LIMIT_PWM_COUNTS (360)

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
#define CHASSIS_LEFT_ENCODER_SIGN             (-1)
#define CHASSIS_RIGHT_ENCODER_SIGN            (1)

#if ((CHASSIS_LEFT_ENCODER_SIGN != 1) && \
    (CHASSIS_LEFT_ENCODER_SIGN != -1))
#error "CHASSIS_LEFT_ENCODER_SIGN must be 1 or -1"
#endif

#if ((CHASSIS_RIGHT_ENCODER_SIGN != 1) && \
    (CHASSIS_RIGHT_ENCODER_SIGN != -1))
#error "CHASSIS_RIGHT_ENCODER_SIGN must be 1 or -1"
#endif

#endif
