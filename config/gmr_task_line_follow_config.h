#ifndef GMR_TASK_LINE_FOLLOW_CONFIG_H
#define GMR_TASK_LINE_FOLLOW_CONFIG_H

/* TASK子菜单第1项的循迹参数；状态机内部沿用Mission2编号。 */
#define GMR_TASK1_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD  (50)
#define GMR_TASK1_LINE_FOLLOW_MIN_SPEED_COUNTS_PER_PERIOD   (10)
#define GMR_TASK1_LINE_FOLLOW_MAX_SPEED_COUNTS_PER_PERIOD   (60)
#define GMR_TASK1_LINE_FOLLOW_GAIN                           (2)
#define GMR_TASK1_LINE_FOLLOW_CORRECTION_LIMIT_COUNTS       (15)
#define GMR_TASK1_LINE_FOLLOW_LOST_HOLD_MS                (2000U)
#define GMR_TASK1_LINE_FOLLOW_S1_WEIGHT                     (-10)
#define GMR_TASK1_LINE_FOLLOW_S2_WEIGHT                      (-6)
#define GMR_TASK1_LINE_FOLLOW_S3_WEIGHT                      (-3)
#define GMR_TASK1_LINE_FOLLOW_S4_WEIGHT                       (0)
#define GMR_TASK1_LINE_FOLLOW_S5_WEIGHT                       (0)
#define GMR_TASK1_LINE_FOLLOW_S6_WEIGHT                       (3)
#define GMR_TASK1_LINE_FOLLOW_S7_WEIGHT                       (6)
#define GMR_TASK1_LINE_FOLLOW_S8_WEIGHT                      (10)

/*
 * Task4/5共用纵向速度斜坡，单位为(count/20ms)/s。
 * 基础速度30时，加速到满速和从满速减到0都需要约3秒。
 * DECEL_LEAD表示在Flash目标里程之前多少count开始减速；当前理论制动距离为2250count。
 */
#define GMR_TASK45_LINE_FOLLOW_ACCELERATION_UNITS_PER_SECOND  (10U)
#define GMR_TASK45_LINE_FOLLOW_DECELERATION_UNITS_PER_SECOND  (10U)
#define GMR_TASK45_LINE_FOLLOW_DECEL_LEAD_COUNTS             (2250U)

/* Task4使用独立循迹参数，当前值仅作为初始安全值。 */
#define GMR_TASK4_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD  (40)
#define GMR_TASK4_LINE_FOLLOW_MIN_SPEED_COUNTS_PER_PERIOD   (10)
#define GMR_TASK4_LINE_FOLLOW_MAX_SPEED_COUNTS_PER_PERIOD   (40)
#define GMR_TASK4_LINE_FOLLOW_GAIN                           (2)
#define GMR_TASK4_LINE_FOLLOW_CORRECTION_LIMIT_COUNTS       (15)
#define GMR_TASK4_LINE_FOLLOW_LOST_HOLD_MS                (2000U)
#define GMR_TASK4_LINE_FOLLOW_LEFT_STOP_COUNT             (8400L)
#define GMR_TASK4_LINE_FOLLOW_RIGHT_STOP_COUNT            (8400L)
#define GMR_TASK4_LINE_FOLLOW_S1_WEIGHT                     (-10)
#define GMR_TASK4_LINE_FOLLOW_S2_WEIGHT                      (-6)
#define GMR_TASK4_LINE_FOLLOW_S3_WEIGHT                      (-3)
#define GMR_TASK4_LINE_FOLLOW_S4_WEIGHT                       (0)
#define GMR_TASK4_LINE_FOLLOW_S5_WEIGHT                       (0)
#define GMR_TASK4_LINE_FOLLOW_S6_WEIGHT                       (3)
#define GMR_TASK4_LINE_FOLLOW_S7_WEIGHT                       (6)
#define GMR_TASK4_LINE_FOLLOW_S8_WEIGHT                      (10)

/* Task5使用第三套独立循迹参数，便于单独进行实车标定。 */
#define GMR_TASK5_LINE_FOLLOW_BASE_SPEED_COUNTS_PER_PERIOD  (30)
#define GMR_TASK5_LINE_FOLLOW_MIN_SPEED_COUNTS_PER_PERIOD   (10)
#define GMR_TASK5_LINE_FOLLOW_MAX_SPEED_COUNTS_PER_PERIOD   (40)
#define GMR_TASK5_LINE_FOLLOW_GAIN                           (2)
#define GMR_TASK5_LINE_FOLLOW_CORRECTION_LIMIT_COUNTS       (15)
#define GMR_TASK5_LINE_FOLLOW_LOST_HOLD_MS                (2000U)
#define GMR_TASK5_LINE_FOLLOW_LEFT_STOP_COUNT             (35600L)
#define GMR_TASK5_LINE_FOLLOW_RIGHT_STOP_COUNT            (45600L)
#define GMR_TASK5_LINE_FOLLOW_S1_WEIGHT                     (-10)
#define GMR_TASK5_LINE_FOLLOW_S2_WEIGHT                      (-6)
#define GMR_TASK5_LINE_FOLLOW_S3_WEIGHT                      (-3)
#define GMR_TASK5_LINE_FOLLOW_S4_WEIGHT                       (0)
#define GMR_TASK5_LINE_FOLLOW_S5_WEIGHT                       (0)
#define GMR_TASK5_LINE_FOLLOW_S6_WEIGHT                       (3)
#define GMR_TASK5_LINE_FOLLOW_S7_WEIGHT                       (6)
#define GMR_TASK5_LINE_FOLLOW_S8_WEIGHT                      (10)

#endif
