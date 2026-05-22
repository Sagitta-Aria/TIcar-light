#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

typedef enum {
    BOARD_ERROR_NONE = 0x00000000U,
    BOARD_ERROR_OLED_I2C = 0x00000001U,
    BOARD_ERROR_CLOCK = 0x00000002U
} BoardErrorCode;

void Board_Init(void);
void Board_Task(void);
void Board_ReportError(BoardErrorCode error);
uint32_t Board_GetErrors(void);
uint8_t Board_HasFatalError(void);
void Board_DebugLedInit(void);
void Board_DebugLedSet(uint8_t enabled);
void Board_DebugLedToggle(void);
void Board_ShowBootProgress(const char *clkStatus, const char *i2cStatus,
    const char *uartStatus, const char *adcStatus, const char *appStatus);

#endif
