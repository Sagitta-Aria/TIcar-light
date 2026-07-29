#ifndef BLUETOOTH_PROTOCOL_H
#define BLUETOOTH_PROTOCOL_H

#include <stdint.h>

#define BLUETOOTH_PROTOCOL_SOF0              (0xA5U)
#define BLUETOOTH_PROTOCOL_SOF1              (0x5AU)
#define BLUETOOTH_PROTOCOL_VERSION           (1U)
#define BLUETOOTH_PROTOCOL_MAX_PAYLOAD       (16U)
#define BLUETOOTH_PROTOCOL_FRAME_OVERHEAD    (10U)
#define BLUETOOTH_PROTOCOL_MAX_FRAME_SIZE \
    (BLUETOOTH_PROTOCOL_FRAME_OVERHEAD + BLUETOOTH_PROTOCOL_MAX_PAYLOAD)

typedef enum {
    BLUETOOTH_FRAME_HELLO = 1,
    BLUETOOTH_FRAME_HEARTBEAT = 2,
    BLUETOOTH_FRAME_SIGNAL = 3
} BluetoothFrameType;

typedef struct {
    uint8_t type;
    uint8_t sequence;
    uint8_t sourceNode;
    uint8_t targetNode;
    uint8_t payloadLength;
    uint8_t payload[BLUETOOTH_PROTOCOL_MAX_PAYLOAD];
} BluetoothProtocolFrame;

typedef struct {
    uint8_t buffer[BLUETOOTH_PROTOCOL_MAX_FRAME_SIZE];
    uint8_t count;
    uint8_t expectedLength;
    uint32_t crcErrorCount;
    uint32_t formatErrorCount;
} BluetoothProtocolParser;

uint16_t BluetoothProtocol_Crc16(const uint8_t *data, uint8_t length);
uint8_t BluetoothProtocol_Encode(const BluetoothProtocolFrame *frame,
    uint8_t *output, uint8_t outputCapacity);
void BluetoothProtocol_ParserInit(BluetoothProtocolParser *parser);
uint8_t BluetoothProtocol_ParserPush(BluetoothProtocolParser *parser,
    uint8_t data, BluetoothProtocolFrame *frame);

#endif
