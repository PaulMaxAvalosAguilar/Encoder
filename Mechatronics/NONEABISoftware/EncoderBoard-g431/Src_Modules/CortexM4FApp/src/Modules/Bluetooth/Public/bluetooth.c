#include "bluetooth.h"
#include "../Internal/Protocol.h"
#include <string.h>

void bluetoothSendEncoderData(uint16_t rom, uint16_t meanPropVelocity, uint16_t peakVelocity)
{
    char payload[6 + 1];
    char romBuffer[2 + 1];
    char meanPropVelocityBuffer[2 + 1];
    char peakVelocityBuffer[2 + 1];
    encodeTwoBytes(rom, romBuffer);
    encodeTwoBytes(meanPropVelocity, meanPropVelocityBuffer);
    encodeTwoBytes(peakVelocity, peakVelocityBuffer);

    strcpy(payload, romBuffer);
    strcat(payload, meanPropVelocityBuffer);
    strcat(payload, peakVelocityBuffer);

    generateProtocolMessage(InternalBluetoothProtocol_MT_EncoderData, payload);
}

void bluetoothSendIsControllerConnected(uint8_t isControllerConnected)
{
    char payload[1 + 1];
    char isControllerConnectedBuffer[1 + 1];
    encodeOneByte(isControllerConnected, isControllerConnectedBuffer);

    strcpy(payload, isControllerConnectedBuffer);

    generateProtocolMessage(InternalBluetoothProtocol_MT_EncoderData, payload);
}

void bluetoothSendIsEncoderTaskStarted(uint8_t isEncoderTaskStarted)
{
    char payload[1 + 1];
    char isEncoderTaskStartedBuffer[1 + 1];
    encodeOneByte(isEncoderTaskStarted, isEncoderTaskStartedBuffer);

    strcpy(payload, isEncoderTaskStartedBuffer);

    generateProtocolMessage(InternalBluetoothProtocol_MT_EncoderTaskStarted, payload);
}

void bluetoothSendIsEncoderTaskStopped(uint8_t isEncoderTaskStopped)
{
    char payload[1 + 1];
    char isEncoderTaskStoppedBuffer[1 + 1];
    encodeOneByte(isEncoderTaskStopped, isEncoderTaskStoppedBuffer);

    strcpy(payload, isEncoderTaskStoppedBuffer);

    generateProtocolMessage(InternalBluetoothProtocol_MT_EncoderTaskStopped, payload);
}

void bluetoothSendBatteryLevel(uint16_t batteryLevel)
{
    char payload[2 + 1];
    char batteryLevelBuffer[2 + 1];
    encodeTwoBytes(batteryLevel, batteryLevelBuffer);

    strcpy(payload, batteryLevelBuffer);

    generateProtocolMessage(InternalBluetoothProtocol_MT_BatteryLevel, payload);
}

void bluetoothSendChargingStatus(uint8_t isEncoderCharging)
{
    char payload[1 + 1];
    char isEncoderChargingBuffer[1 + 1];
    encodeTwoBytes(isEncoderCharging, isEncoderChargingBuffer);

    strcpy(payload, isEncoderChargingBuffer);

    generateProtocolMessage(InternalBluetoothProtocol_MT_ChargingStatus, payload);
}