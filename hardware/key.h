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
    KEY_EVENT_2
} KeyEvent;

void Key_Init(void);
void Key_Task(void);
uint8_t Key_IsPressed(KeyId key);
KeyEvent Key_PopEvent(void);
void Key_HandleGPIOInterrupt(void);

#endif
