#ifndef BOARD_H
#define BOARD_H

void Board_Init(void);
void Board_Task(void);
void Board_ShowBootProgress(const char *clkStatus, const char *i2cStatus,
    const char *uartStatus, const char *adcStatus, const char *appStatus);

#endif
