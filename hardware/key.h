#ifndef KEY_H
#define KEY_H

#include <stdint.h>

typedef enum {
    KEY_ID_1 = 0,
    KEY_ID_2,
    KEY_ID_COUNT
} KeyId;

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_1,
    KEY_EVENT_2,
    KEY_EVENT_1_LONG,
    KEY_EVENT_2_LONG
} KeyEvent;

void Key_Init(void);
void Key_Task(void);
uint8_t Key_IsPressed(KeyId key);
uint8_t Key_HasPendingEvent(void);
KeyEvent Key_PopEvent(void);
uint8_t Key_HandleGPIOInterrupt(void);

#endif
