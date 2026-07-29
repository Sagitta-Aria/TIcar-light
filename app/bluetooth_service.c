#include "bluetooth_service.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "bluetooth_config.h"
#include "bluetooth_uart.h"

#define BLUETOOTH_SERVICE_QUEUE_LENGTH      (8U)

static StaticQueue_t g_txQueueControl;
static uint8_t g_txQueueStorage[
    BLUETOOTH_SERVICE_QUEUE_LENGTH * sizeof(BluetoothSignal)];
static QueueHandle_t g_txQueue;
static StaticQueue_t g_rxQueueControl;
static uint8_t g_rxQueueStorage[
    BLUETOOTH_SERVICE_QUEUE_LENGTH * sizeof(BluetoothSignal)];
static QueueHandle_t g_rxQueue;
static uint8_t g_wasConnected;
static uint32_t g_serviceRxDropCount;
static uint32_t g_serviceTxDropCount;

void BluetoothService_Init(void)
{
    g_txQueue = 0;
    g_rxQueue = 0;
    g_wasConnected = 0U;
    g_serviceRxDropCount = 0U;
    g_serviceTxDropCount = 0U;
    BluetoothLink_Init();
#if CAR_BLUETOOTH_ENABLED
    g_txQueue = xQueueCreateStatic(BLUETOOTH_SERVICE_QUEUE_LENGTH,
        sizeof(BluetoothSignal), g_txQueueStorage, &g_txQueueControl);
    g_rxQueue = xQueueCreateStatic(BLUETOOTH_SERVICE_QUEUE_LENGTH,
        sizeof(BluetoothSignal), g_rxQueueStorage, &g_rxQueueControl);
    configASSERT(g_txQueue != 0);
    configASSERT(g_rxQueue != 0);
#endif
    BluetoothUart_Init();
}

void BluetoothService_Task(uint32_t elapsedMs)
{
#if CAR_BLUETOOTH_ENABLED
    BluetoothSignal signal;
    uint8_t connected;

    BluetoothLink_Task(elapsedMs);
    connected = BluetoothLink_IsAuthenticated();
    if ((connected == 0U) && (g_wasConnected != 0U) && (g_txQueue != 0)) {
        g_serviceTxDropCount += (uint32_t)uxQueueMessagesWaiting(g_txQueue);
        (void)xQueueReset(g_txQueue);
    }
    g_wasConnected = connected;
    while ((g_txQueue != 0) &&
        (xQueuePeek(g_txQueue, &signal, 0U) == pdPASS)) {
        if (BluetoothLink_QueueSignal(&signal) == 0U) {
            break;
        }
        (void)xQueueReceive(g_txQueue, &signal, 0U);
    }
    while (BluetoothLink_TakeSignal(&signal) != 0U) {
        if ((g_rxQueue == 0) ||
            (xQueueSendToBack(g_rxQueue, &signal, 0U) != pdPASS)) {
            ++g_serviceRxDropCount;
            break;
        }
    }
    BluetoothUart_Task();
#else
    (void)elapsedMs;
#endif
}

uint8_t BluetoothService_SendSignal(uint16_t id, int32_t value,
    uint8_t flags)
{
#if CAR_BLUETOOTH_ENABLED
    BluetoothSignal signal;

    if ((g_txQueue == 0) || (BluetoothLink_IsAuthenticated() == 0U)) {
        ++g_serviceTxDropCount;
        return 0U;
    }
    signal.id = id;
    signal.value = value;
    signal.flags = flags;
    signal.sequence = 0U;
    if (xQueueSendToBack(g_txQueue, &signal, 0U) != pdPASS) {
        ++g_serviceTxDropCount;
        return 0U;
    }
    return 1U;
#else
    (void)id;
    (void)value;
    (void)flags;
    return 0U;
#endif
}

uint8_t BluetoothService_TakeSignal(BluetoothSignal *signal)
{
#if CAR_BLUETOOTH_ENABLED
    if ((signal == 0) || (g_rxQueue == 0)) {
        return 0U;
    }
    return (uint8_t)((xQueueReceive(g_rxQueue, signal, 0U) == pdPASS) ?
        1U : 0U);
#else
    (void)signal;
    return 0U;
#endif
}

uint8_t BluetoothService_IsConnected(void)
{
    return BluetoothLink_IsAuthenticated();
}

void BluetoothService_GetStatus(BluetoothLinkStatus *status)
{
    BluetoothLink_GetStatus(status);
    if (status != 0) {
        status->rxDropCount += g_serviceRxDropCount;
        status->txDropCount += g_serviceTxDropCount;
    }
}

void BluetoothService_InputByteFromISR(uint8_t data)
{
    BluetoothLink_InputByteFromISR(data);
}

uint8_t BluetoothService_TakeTxByteFromISR(uint8_t *data)
{
    return BluetoothLink_TakeTxByteFromISR(data);
}
