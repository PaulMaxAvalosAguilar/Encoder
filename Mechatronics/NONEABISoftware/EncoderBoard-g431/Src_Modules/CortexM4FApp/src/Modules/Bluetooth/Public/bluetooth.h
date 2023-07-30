#pragma once

#include <inttypes.h>

void bluetoothSendEncoderData(uint16_t rom, uint16_t meanPropVelocity, uint16_t peakVelocity);
void bluetoothSendIsControllerConnected(uint8_t isControllerConnected);
void bluetoothSendIsEncoderTaskStarted(uint8_t isEncoderTaskStarted);
void bluetoothSendIsEncoderTaskStopped(uint8_t isEncoderTaskStopped);
void bluetoothSendBatteryLevel(uint16_t batteryLevel);
void bluetoothSendChargingStatus(uint8_t isEncoderCharging);
