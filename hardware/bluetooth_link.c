#include "bluetooth_link.h"

#include <string.h>

#include "bluetooth_config.h"
#include "bluetooth_protocol.h"

#define BLUETOOTH_LINK_MAC_LENGTH            (6U)
#define BLUETOOTH_LINK_HELLO_LENGTH          (13U)
#define BLUETOOTH_LINK_SIGNAL_LENGTH         (7U)
#define BLUETOOTH_LINK_BYTE_QUEUE_LENGTH     (256U)
#define BLUETOOTH_LINK_SIGNAL_QUEUE_LENGTH   (8U)

static const uint8_t g_localMac[BLUETOOTH_LINK_MAC_LENGTH] = {
    CAR_BLUETOOTH_LOCAL_MAC_BYTES
};
static const uint8_t g_peerMac[BLUETOOTH_LINK_MAC_LENGTH] = {
    CAR_BLUETOOTH_PEER_MAC_BYTES
};

static volatile uint16_t g_rxHead;
static volatile uint16_t g_rxTail;
static uint8_t g_rxBytes[BLUETOOTH_LINK_BYTE_QUEUE_LENGTH];
static volatile uint16_t g_txHead;
static volatile uint16_t g_txTail;
static uint8_t g_txBytes[BLUETOOTH_LINK_BYTE_QUEUE_LENGTH];
static BluetoothSignal g_rxSignals[BLUETOOTH_LINK_SIGNAL_QUEUE_LENGTH];
static uint8_t g_signalHead;
static uint8_t g_signalTail;
static uint8_t g_txSequence;
static uint32_t g_helloElapsedMs;
static uint32_t g_validRxElapsedMs;
static BluetoothProtocolParser g_parser;
static BluetoothLinkStatus g_status;

static uint32_t BluetoothLink_SaturatingAdd(uint32_t value, uint32_t increment)
{
    if (value > (0xFFFFFFFFU - increment)) {
        return 0xFFFFFFFFU;
    }
    return value + increment;
}

static uint16_t BluetoothLink_NextByteIndex(uint16_t index)
{
    return (uint16_t)((index + 1U) % BLUETOOTH_LINK_BYTE_QUEUE_LENGTH);
}

static uint8_t BluetoothLink_PopRxByte(uint8_t *data)
{
    uint16_t tail;

    if ((data == 0) || (g_rxTail == g_rxHead)) {
        return 0U;
    }
    tail = g_rxTail;
    *data = g_rxBytes[tail];
    g_rxTail = BluetoothLink_NextByteIndex(tail);
    return 1U;
}

static uint16_t BluetoothLink_GetTxFree(void)
{
    uint16_t head = g_txHead;
    uint16_t tail = g_txTail;

    if (head >= tail) {
        return (uint16_t)(BLUETOOTH_LINK_BYTE_QUEUE_LENGTH -
            (head - tail) - 1U);
    }
    return (uint16_t)(tail - head - 1U);
}

static uint8_t BluetoothLink_QueueFrame(const BluetoothProtocolFrame *frame)
{
    uint8_t encoded[BLUETOOTH_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t length;
    uint8_t index;
    uint16_t head;

    length = BluetoothProtocol_Encode(frame, encoded,
        (uint8_t)sizeof(encoded));
    if ((length == 0U) || (BluetoothLink_GetTxFree() < length)) {
        return 0U;
    }
    head = g_txHead;
    for (index = 0U; index < length; ++index) {
        g_txBytes[head] = encoded[index];
        head = BluetoothLink_NextByteIndex(head);
    }
    g_txHead = head;
    ++g_status.txFrameCount;
    return 1U;
}

static uint8_t BluetoothLink_QueueHello(void)
{
    BluetoothProtocolFrame frame;

    memset(&frame, 0, sizeof(frame));
    frame.type = BLUETOOTH_FRAME_HELLO;
    frame.sequence = g_txSequence++;
    frame.sourceNode = (uint8_t)CAR_BLUETOOTH_LOCAL_NODE_ID;
    frame.targetNode = (uint8_t)CAR_BLUETOOTH_PEER_NODE_ID;
    frame.payloadLength = BLUETOOTH_LINK_HELLO_LENGTH;
    frame.payload[0] = (uint8_t)CAR_BLUETOOTH_ROLE;
    memcpy(&frame.payload[1], g_localMac, BLUETOOTH_LINK_MAC_LENGTH);
    memcpy(&frame.payload[7], g_peerMac, BLUETOOTH_LINK_MAC_LENGTH);
    return BluetoothLink_QueueFrame(&frame);
}

static uint8_t BluetoothLink_IsPeerFrame(const BluetoothProtocolFrame *frame)
{
    return (uint8_t)(((frame->sourceNode ==
        (uint8_t)CAR_BLUETOOTH_PEER_NODE_ID) &&
        (frame->targetNode == (uint8_t)CAR_BLUETOOTH_LOCAL_NODE_ID)) ? 1U : 0U);
}

static uint8_t BluetoothLink_AuthenticateHello(
    const BluetoothProtocolFrame *frame)
{
    if ((frame->payloadLength != BLUETOOTH_LINK_HELLO_LENGTH) ||
        (frame->payload[0] != (uint8_t)CAR_BLUETOOTH_PEER_ROLE) ||
        (memcmp(&frame->payload[1], g_peerMac,
            BLUETOOTH_LINK_MAC_LENGTH) != 0) ||
        (memcmp(&frame->payload[7], g_localMac,
            BLUETOOTH_LINK_MAC_LENGTH) != 0)) {
        ++g_status.identityErrorCount;
        g_status.authenticated = 0U;
        return 0U;
    }
    g_status.authenticated = 1U;
    g_validRxElapsedMs = 0U;
    return 1U;
}

static void BluetoothLink_StoreSignal(const BluetoothProtocolFrame *frame)
{
    BluetoothSignal signal;
    uint8_t nextHead = (uint8_t)((g_signalHead + 1U) %
        BLUETOOTH_LINK_SIGNAL_QUEUE_LENGTH);
    uint32_t rawValue;

    if (frame->payloadLength != BLUETOOTH_LINK_SIGNAL_LENGTH) {
        ++g_status.formatErrorCount;
        return;
    }
    if (nextHead == g_signalTail) {
        ++g_status.rxDropCount;
        return;
    }
    signal.id = (uint16_t)frame->payload[0];
    signal.id |= (uint16_t)((uint16_t)frame->payload[1] << 8U);
    rawValue = (uint32_t)frame->payload[2];
    rawValue |= (uint32_t)((uint32_t)frame->payload[3] << 8U);
    rawValue |= (uint32_t)((uint32_t)frame->payload[4] << 16U);
    rawValue |= (uint32_t)((uint32_t)frame->payload[5] << 24U);
    signal.value = (int32_t)rawValue;
    signal.flags = frame->payload[6];
    signal.sequence = frame->sequence;
    g_rxSignals[g_signalHead] = signal;
    g_signalHead = nextHead;
    ++g_status.rxSignalCount;
}

static void BluetoothLink_HandleFrame(const BluetoothProtocolFrame *frame)
{
    ++g_status.rxFrameCount;
    if (BluetoothLink_IsPeerFrame(frame) == 0U) {
        ++g_status.identityErrorCount;
        return;
    }
    if (frame->type == BLUETOOTH_FRAME_HELLO) {
        (void)BluetoothLink_AuthenticateHello(frame);
        return;
    }
    if (g_status.authenticated == 0U) {
        ++g_status.identityErrorCount;
        return;
    }
    g_validRxElapsedMs = 0U;
    if (frame->type == BLUETOOTH_FRAME_SIGNAL) {
        BluetoothLink_StoreSignal(frame);
    } else if (frame->type != BLUETOOTH_FRAME_HEARTBEAT) {
        ++g_status.formatErrorCount;
    }
}

void BluetoothLink_Init(void)
{
    memset(&g_status, 0, sizeof(g_status));
    g_status.enabled = (uint8_t)(CAR_BLUETOOTH_ENABLED ? 1U : 0U);
    g_status.role = (uint8_t)CAR_BLUETOOTH_ROLE;
    g_status.localNode = (uint8_t)CAR_BLUETOOTH_LOCAL_NODE_ID;
    g_status.peerNode = (uint8_t)CAR_BLUETOOTH_PEER_NODE_ID;
    g_rxHead = 0U;
    g_rxTail = 0U;
    g_txHead = 0U;
    g_txTail = 0U;
    g_signalHead = 0U;
    g_signalTail = 0U;
    g_txSequence = 0U;
    g_helloElapsedMs = CAR_BLUETOOTH_HELLO_PERIOD_MS;
    g_validRxElapsedMs = 0U;
    BluetoothProtocol_ParserInit(&g_parser);
}

void BluetoothLink_Task(uint32_t elapsedMs)
{
    BluetoothProtocolFrame frame;
    uint8_t data;

    if (g_status.enabled == 0U) {
        return;
    }
    while (BluetoothLink_PopRxByte(&data) != 0U) {
        if (BluetoothProtocol_ParserPush(&g_parser, data, &frame) != 0U) {
            BluetoothLink_HandleFrame(&frame);
        }
    }
    g_status.crcErrorCount = g_parser.crcErrorCount;
    g_status.formatErrorCount = BluetoothLink_SaturatingAdd(
        g_status.formatErrorCount, g_parser.formatErrorCount);
    g_parser.formatErrorCount = 0U;

    g_helloElapsedMs = BluetoothLink_SaturatingAdd(g_helloElapsedMs,
        elapsedMs);
    g_validRxElapsedMs = BluetoothLink_SaturatingAdd(g_validRxElapsedMs,
        elapsedMs);
    if ((g_status.authenticated != 0U) &&
        (g_validRxElapsedMs >= CAR_BLUETOOTH_LINK_TIMEOUT_MS)) {
        g_status.authenticated = 0U;
        ++g_status.timeoutCount;
    }
    if (g_helloElapsedMs >= CAR_BLUETOOTH_HELLO_PERIOD_MS) {
        g_helloElapsedMs = 0U;
        (void)BluetoothLink_QueueHello();
    }
}

uint8_t BluetoothLink_QueueSignal(const BluetoothSignal *signal)
{
    BluetoothProtocolFrame frame;
    uint32_t rawValue;

    if ((signal == 0) || (g_status.enabled == 0U) ||
        (g_status.authenticated == 0U)) {
        return 0U;
    }
    memset(&frame, 0, sizeof(frame));
    frame.type = BLUETOOTH_FRAME_SIGNAL;
    frame.sequence = g_txSequence++;
    frame.sourceNode = (uint8_t)CAR_BLUETOOTH_LOCAL_NODE_ID;
    frame.targetNode = (uint8_t)CAR_BLUETOOTH_PEER_NODE_ID;
    frame.payloadLength = BLUETOOTH_LINK_SIGNAL_LENGTH;
    frame.payload[0] = (uint8_t)(signal->id & 0xFFU);
    frame.payload[1] = (uint8_t)(signal->id >> 8U);
    rawValue = (uint32_t)signal->value;
    frame.payload[2] = (uint8_t)(rawValue & 0xFFU);
    frame.payload[3] = (uint8_t)((rawValue >> 8U) & 0xFFU);
    frame.payload[4] = (uint8_t)((rawValue >> 16U) & 0xFFU);
    frame.payload[5] = (uint8_t)((rawValue >> 24U) & 0xFFU);
    frame.payload[6] = signal->flags;
    if (BluetoothLink_QueueFrame(&frame) == 0U) {
        return 0U;
    }
    ++g_status.txSignalCount;
    return 1U;
}

uint8_t BluetoothLink_TakeSignal(BluetoothSignal *signal)
{
    if ((signal == 0) || (g_signalTail == g_signalHead)) {
        return 0U;
    }
    *signal = g_rxSignals[g_signalTail];
    g_signalTail = (uint8_t)((g_signalTail + 1U) %
        BLUETOOTH_LINK_SIGNAL_QUEUE_LENGTH);
    return 1U;
}

uint8_t BluetoothLink_IsAuthenticated(void)
{
    return g_status.authenticated;
}

void BluetoothLink_GetStatus(BluetoothLinkStatus *status)
{
    if (status != 0) {
        *status = g_status;
    }
}

void BluetoothLink_InputByteFromISR(uint8_t data)
{
    uint16_t head;
    uint16_t nextHead;

    if (g_status.enabled == 0U) {
        return;
    }
    head = g_rxHead;
    nextHead = BluetoothLink_NextByteIndex(head);
    if (nextHead == g_rxTail) {
        ++g_status.rxDropCount;
        return;
    }
    g_rxBytes[head] = data;
    g_rxHead = nextHead;
}

uint8_t BluetoothLink_TakeTxByteFromISR(uint8_t *data)
{
    uint16_t tail;

    if ((data == 0) || (g_txTail == g_txHead)) {
        return 0U;
    }
    tail = g_txTail;
    *data = g_txBytes[tail];
    g_txTail = BluetoothLink_NextByteIndex(tail);
    return 1U;
}
