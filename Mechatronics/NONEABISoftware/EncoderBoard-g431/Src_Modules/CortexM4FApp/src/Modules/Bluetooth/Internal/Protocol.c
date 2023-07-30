#include "Protocol.h"
#include "deps.h"
#include <string.h>

void encodeTwoBytes(uint16_t numberToEncode, char *twoByteBuffer)
{
    uint8_t lowPart = 0;
    uint8_t highPart = 0;

    lowPart = ((numberToEncode & 0x7F) << 1) | 1;
    highPart = (numberToEncode >> 6) | 1;
    twoByteBuffer[0] = highPart;
    twoByteBuffer[1] = lowPart;
    twoByteBuffer[2] = 0;
}

void encodeOneByte(uint16_t numberToEncode, char *oneByteBuffer)
{
    uint8_t lowPart = 0;
    lowPart = (numberToEncode << 1) | 1;
    oneByteBuffer[0] = (uint8_t)lowPart;
    oneByteBuffer[1] = 0;
}

uint16_t decodeTwoBytes(uint8_t msb, uint8_t lsb)
{
    return (lsb >> 1) | ((msb & 0xFE) << 6);
}

uint8_t decodeOneByte(uint8_t byte)
{
    return byte >> 1;
}

void generateProtocolMessage(const char protocolMessageType, const char *payloadString)
{
    char initByteSequence[3] = {PROTOCOL_INITIATOR, protocolMessageType, '\0'};
    char endingByteSequence[2] = {PROTOCOL_TERMINATOR, '\0'};

    size_t messageSize = 0;
    messageSize += strlen(payloadString);
    messageSize += 3;

    char protocolMessage[messageSize];
    strcpy(protocolMessage, initByteSequence);
    strcat(protocolMessage, payloadString);
    strcat(protocolMessage, endingByteSequence);

    BLUETOOTH_TRANSMIT((uint8_t *)protocolMessage, messageSize);
}