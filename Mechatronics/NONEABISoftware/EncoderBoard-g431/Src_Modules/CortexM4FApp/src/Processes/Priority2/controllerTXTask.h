#include "FreeRTOS.h"

#pragma once

// Task Parameters-----------------------------------------------

#define CONTROLLERTX_TASK_STACK_SIZE 128
#define CONTROLLERTX_TASK_STACK_PRIORITY 2

// Task Manipulation---------------------------------------------
void __controllerTXTask_init(void);

// IPC: Inter Process Comumnication------------------------------
#define CONTROLLERTX_TASK_QUEUE_SIZE 20

typedef enum // IPCMT: Inter Process Communication Message Type
{
    ControllerTXTask_IPMCT_01_SendEncoderData,
    ControllerTXTask_IPMCT_02_ControllerConnectionStatus,
    ControllerTXTask_IPMCT_03_ConfirmEncoderTaskStarted,
    ControllerTXTask_IPMCT_04_ConfirmEncoderTaskStoped,
    ControllerTXTask_IPMCT_05_SendBatteryLevel,
    ControllerTXTask_IPMCT_06_SendChargingStatus
} ControllerTXTask_IPC_Message_Type;

typedef struct
{
    uint16_t rom;
    uint16_t meanPropVelocity;
    uint16_t peakVelocity;
} EncoderDataType;

typedef struct
{
    // IPCMT: Inter Process Communication Message Type
    ControllerTXTask_IPC_Message_Type batteryTask_IPCMT;
    union
    {
        EncoderDataType _01_encoderData;
        uint8_t _02_isControllerConnected;
        uint8_t _03_isEncoderTaskStarted;
        uint8_t _04_isEncoderTaskStopped;
        uint16_t _05_batteryLevel;
        uint8_t _06_isEncoderCharging;
    } payload;
} ControllerTXTask_IPC_Message_Struct;

void __IPC_controllerTXTask_SendMessage_SendEncoderData(uint16_t rom,
                                                        uint16_t meanPropVelocity,
                                                        uint16_t peakVelocity);
void __IPC_controllerTXTask_SendMessage_ControllerConnectionStatus(uint8_t isControllerConected);
void __IPC_controllerTXTask_SendMessage_ConfirmEncoderTaskStarted(uint8_t isEncoderTaskStarted);
void __IPC_controllerTXTask_SendMessage_ConfirmEncoderTaskStoped(uint8_t isEncoderTaskStopped);
void __IPC_controllerTXTask_SendMessage_SendBatteryLevel(uint16_t batteryLevel);
void __IPC_controllerTXTask_SendMessage_SendChargingStatus(uint8_t isEncoderCharging);
