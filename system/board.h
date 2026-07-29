#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

/* 板级初始化错误位，可按位组合；fatal错误会阻止进入比赛任务。 */
typedef enum {
    BOARD_ERROR_NONE = 0x00000000U,
    BOARD_ERROR_OLED_I2C = 0x00000001U,
    BOARD_ERROR_CLOCK = 0x00000002U,
    BOARD_ERROR_IMU660RX = 0x00000004U
} BoardErrorCode;

/* 按固定顺序初始化SysConfig外设、日志、LCD、传感器和电机；只在main调用一次。 */
void Board_Init(void);

/* UI任务的板级维护入口；完成一次心跳维护时返回1。 */
uint8_t Board_Task(void);

/* 记录板级错误位并更新调试灯；可重复调用，不会清除旧错误。 */
void Board_ReportError(BoardErrorCode error);

/* 读取累计错误位和上电锁存的SYSCTL复位原因。 */
uint32_t Board_GetErrors(void);
uint32_t Board_GetResetCause(void);

/* 存在不允许继续比赛的板级错误时返回1。 */
uint8_t Board_HasFatalError(void);

/* 任意一个已选择显示后端准备好时返回1；全部关闭或初始化失败时返回0。 */
uint8_t Board_IsDisplayAvailable(void);

/* PA14调试灯接口；Init后才允许Set/Toggle，禁止用于电机时序。 */
void Board_DebugLedInit(void);
void Board_DebugLedSet(uint8_t enabled);
void Board_DebugLedToggle(void);

/* 启动阶段向H7 LCD写入各子系统状态文本；不能在ISR内调用。 */
void Board_ShowBootProgress(const char *clkStatus, const char *i2cStatus,
    const char *uartStatus, const char *adcStatus, const char *appStatus);

#endif
