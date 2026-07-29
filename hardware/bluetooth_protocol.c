#include "bluetooth_protocol.h"

#include <string.h>

#define BLUETOOTH_PROTOCOL_HEADER_SIZE       (8U)
#define BLUETOOTH_PROTOCOL_CRC_SIZE          (2U)

uint16_t BluetoothProtocol_Crc16(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t byteIndex;
    uint8_t bitIndex;

    if (data == 0) {
        return 0U;
    }
    for (byteIndex = 0U; byteIndex < length; ++byteIndex) {
        crc ^= (uint16_t)((uint16_t)data[byteIndex] << 8U);
        for (bitIndex = 0U; bitIndex < 8U; ++bitIndex) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

uint8_t BluetoothProtocol_Encode(const BluetoothProtocolFrame *frame,
    uint8_t *output, uint8_t outputCapacity)
{
    uint8_t frameLength;
    uint16_t crc;

    if ((frame == 0) || (output == 0) ||
        (frame->payloadLength > BLUETOOTH_PROTOCOL_MAX_PAYLOAD)) {
        return 0U;
    }
    frameLength = (uint8_t)(BLUETOOTH_PROTOCOL_FRAME_OVERHEAD +
        frame->payloadLength);
    if (outputCapacity < frameLength) {
        return 0U;
    }

    output[0] = BLUETOOTH_PROTOCOL_SOF0;
    output[1] = BLUETOOTH_PROTOCOL_SOF1;
    output[2] = BLUETOOTH_PROTOCOL_VERSION;
    output[3] = frame->type;
    output[4] = frame->sequence;
    output[5] = frame->sourceNode;
    output[6] = frame->targetNode;
    output[7] = frame->payloadLength;
    if (frame->payloadLength != 0U) {
        memcpy(&output[BLUETOOTH_PROTOCOL_HEADER_SIZE], frame->payload,
            frame->payloadLength);
    }
    crc = BluetoothProtocol_Crc16(&output[2],
        (uint8_t)(6U + frame->payloadLength));
    output[(uint8_t)(BLUETOOTH_PROTOCOL_HEADER_SIZE + frame->payloadLength)] =
        (uint8_t)(crc & 0xFFU);
    output[(uint8_t)(BLUETOOTH_PROTOCOL_HEADER_SIZE +
        frame->payloadLength + 1U)] = (uint8_t)(crc >> 8U);
    return frameLength;
}

void BluetoothProtocol_ParserInit(BluetoothProtocolParser *parser)
{
    if (parser != 0) {
        memset(parser, 0, sizeof(*parser));
    }
}

static void BluetoothProtocol_ParserReset(BluetoothProtocolParser *parser)
{
    parser->count = 0U;
    parser->expectedLength = 0U;
}

uint8_t BluetoothProtocol_ParserPush(BluetoothProtocolParser *parser,
    uint8_t data, BluetoothProtocolFrame *frame)
{
    uint8_t payloadLength;
    uint16_t expectedCrc;
    uint16_t actualCrc;

    if ((parser == 0) || (frame == 0)) {
        return 0U;
    }
    if (parser->count == 0U) {
        if (data == BLUETOOTH_PROTOCOL_SOF0) {
            parser->buffer[0] = data;
            parser->count = 1U;
        }
        return 0U;
    }
    if (parser->count == 1U) {
        if (data == BLUETOOTH_PROTOCOL_SOF1) {
            parser->buffer[1] = data;
            parser->count = 2U;
        } else if (data != BLUETOOTH_PROTOCOL_SOF0) {
            BluetoothProtocol_ParserReset(parser);
        }
        return 0U;
    }

    parser->buffer[parser->count++] = data;
    if ((parser->count == 3U) &&
        (parser->buffer[2] != BLUETOOTH_PROTOCOL_VERSION)) {
        ++parser->formatErrorCount;
        BluetoothProtocol_ParserReset(parser);
        return 0U;
    }
    if (parser->count == BLUETOOTH_PROTOCOL_HEADER_SIZE) {
        payloadLength = parser->buffer[7];
        if (payloadLength > BLUETOOTH_PROTOCOL_MAX_PAYLOAD) {
            ++parser->formatErrorCount;
            BluetoothProtocol_ParserReset(parser);
            return 0U;
        }
        parser->expectedLength = (uint8_t)(BLUETOOTH_PROTOCOL_FRAME_OVERHEAD +
            payloadLength);
    }
    if ((parser->expectedLength == 0U) ||
        (parser->count < parser->expectedLength)) {
        return 0U;
    }

    payloadLength = parser->buffer[7];
    expectedCrc = (uint16_t)parser->buffer[
        (uint8_t)(BLUETOOTH_PROTOCOL_HEADER_SIZE + payloadLength)];
    expectedCrc |= (uint16_t)((uint16_t)parser->buffer[
        (uint8_t)(BLUETOOTH_PROTOCOL_HEADER_SIZE + payloadLength + 1U)] << 8U);
    actualCrc = BluetoothProtocol_Crc16(&parser->buffer[2],
        (uint8_t)(6U + payloadLength));
    if (expectedCrc != actualCrc) {
        ++parser->crcErrorCount;
        BluetoothProtocol_ParserReset(parser);
        return 0U;
    }

    frame->type = parser->buffer[3];
    frame->sequence = parser->buffer[4];
    frame->sourceNode = parser->buffer[5];
    frame->targetNode = parser->buffer[6];
    frame->payloadLength = payloadLength;
    if (payloadLength != 0U) {
        memcpy(frame->payload,
            &parser->buffer[BLUETOOTH_PROTOCOL_HEADER_SIZE], payloadLength);
    }
    BluetoothProtocol_ParserReset(parser);
    return 1U;
}
