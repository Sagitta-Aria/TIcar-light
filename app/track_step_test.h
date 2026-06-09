#ifndef TRACK_STEP_TEST_H
#define TRACK_STEP_TEST_H

#include <stdint.h>

/* TrackStepTest_Init：初始化底盘电机 SPS 测试状态，默认不输出电机命令。 */
void TrackStepTest_Init(void);

/*
 * TrackStepTest_Start：开始底盘无限循迹测试。
 * 说明：清零底盘左右 STEP 计数，然后按灰度状态持续更新底盘命令。
 */
void TrackStepTest_Start(void);

/* TrackStepTest_Stop：停止底盘无限循迹测试并停止底盘。 */
void TrackStepTest_Stop(void);

/*
 * TrackStepTest_Task：执行一轮灰度无限循迹。
 * 使用场景：状态机处于 Motor 测试时每轮调用。
 */
void TrackStepTest_Task(void);

/* TrackStepTest_IncreaseSpeed/DecreaseSpeed：按 500 SPS 调整底盘基础速度。 */
void TrackStepTest_IncreaseSpeed(void);
void TrackStepTest_DecreaseSpeed(void);

/* TrackStepTest_GetSpeedSps：读取当前底盘基础速度，单位 step/s。 */
uint16_t TrackStepTest_GetSpeedSps(void);

/* TrackStepTest_GetTurnSpeedSps：读取急转时内外轮使用的速度，单位 step/s。 */
uint16_t TrackStepTest_GetTurnSpeedSps(void);

/* TrackStepTest_GetModeName：读取当前循迹模式名称，用于 OLED 状态显示。 */
const char *TrackStepTest_GetModeName(void);

/* TrackStepTest_IsRunning：返回测试是否还在输出底盘速度。 */
uint8_t TrackStepTest_IsRunning(void);

/* TrackStepTest_IsDone：兼容旧显示接口；当前持续测试模式下始终返回 0。 */
uint8_t TrackStepTest_IsDone(void);

/* TrackStepTest_GetProgressPercent：兼容旧显示接口；当前持续测试模式下始终返回 0。 */
uint8_t TrackStepTest_GetProgressPercent(void);

/* TrackStepTest_GetTravelSteps：返回底盘左右平均 STEP 绝对值。 */
uint32_t TrackStepTest_GetTravelSteps(void);

#endif
