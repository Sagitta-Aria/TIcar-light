#ifndef BLUETOOTH_SERVICE_H
#define BLUETOOTH_SERVICE_H

#include <stdint.h>

#include "bluetooth_link.h"

void BluetoothService_Init(void);
void BluetoothService_Task(uint32_t elapsedMs);

/* Task-context API for future vehicle state machines. */
uint8_t BluetoothService_SendSignal(uint16_t id, int32_t value,
    uint8_t flags);
uint8_t BluetoothService_TakeSignal(BluetoothSignal *signal);
uint8_t BluetoothService_IsConnected(void);
void BluetoothService_GetStatus(BluetoothLinkStatus *status);

/* UART transport is intentionally deferred; its ISR will call these APIs. */
void BluetoothService_InputByteFromISR(uint8_t data);
uint8_t BluetoothService_TakeTxByteFromISR(uint8_t *data);

#endif
