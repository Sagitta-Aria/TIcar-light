#ifndef BLUETOOTH_LINK_H
#define BLUETOOTH_LINK_H

#include <stdint.h>

typedef struct {
    uint16_t id;
    int32_t value;
    uint8_t flags;
    uint8_t sequence;
} BluetoothSignal;

typedef struct {
    uint8_t enabled;
    uint8_t role;
    uint8_t authenticated;
    uint8_t localNode;
    uint8_t peerNode;
    uint32_t rxFrameCount;
    uint32_t txFrameCount;
    uint32_t rxSignalCount;
    uint32_t txSignalCount;
    uint32_t identityErrorCount;
    uint32_t crcErrorCount;
    uint32_t formatErrorCount;
    uint32_t rxDropCount;
    uint32_t txDropCount;
    uint32_t timeoutCount;
} BluetoothLinkStatus;

void BluetoothLink_Init(void);
void BluetoothLink_Task(uint32_t elapsedMs);
uint8_t BluetoothLink_QueueSignal(const BluetoothSignal *signal);
uint8_t BluetoothLink_TakeSignal(BluetoothSignal *signal);
uint8_t BluetoothLink_IsAuthenticated(void);
void BluetoothLink_GetStatus(BluetoothLinkStatus *status);

/* Future UART RX/TX interrupt handlers connect only to these byte APIs. */
void BluetoothLink_InputByteFromISR(uint8_t data);
uint8_t BluetoothLink_TakeTxByteFromISR(uint8_t *data);

#endif
