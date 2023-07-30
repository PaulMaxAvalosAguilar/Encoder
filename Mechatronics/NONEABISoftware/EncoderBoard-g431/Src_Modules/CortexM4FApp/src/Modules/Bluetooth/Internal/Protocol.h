#pragma once

#include <inttypes.h>

#define PROTOCOL_INITIATOR '%'
#define PROTOCOL_TERMINATOR '%'

typedef enum
{
    InternalBluetoothProtocol_MT_DummyType = 1,
    InternalBluetoothProtocol_MT_EncoderData = 2,
    InternalBluetoothProtocol_MT_ConnectionStatus = 3,
    InternalBluetoothProtocol_MT_EncoderTaskStarted = 4,
    InternalBluetoothProtocol_MT_EncoderTaskStopped = 5,
    InternalBluetoothProtocol_MT_BatteryLevel = 6,
    InternalBluetoothProtocol_MT_ChargingStatus = 7
} protocolMessageTypes;

void encodeTwoBytes(uint16_t numberToEncode, char *twoByteBuffer);
void encodeOneByte(uint16_t numberToEncode, char *oneByteBuffer);
uint16_t decodeTwoBytes(uint8_t msb, uint8_t lsb);
uint8_t decodeOneByte(uint8_t byte);

void generateProtocolMessage(const char protocolMessageType, const char *payloadString);