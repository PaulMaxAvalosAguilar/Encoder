#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#pragma once

// Task Parameters-----------------------------------------------

#define DISPLAY_TASK_STACK_SIZE 256
#define DISPLAY_TASK_PRIORITY 2

void __displayTask_init(void);

// IPC: Inter Process Comumnication------------------------------
#define DISPLAY_TASK_QUEUE_SIZE 20
extern QueueHandle_t displayTask_QueueHandle;

typedef enum // IPCMT: Inter Process Communication Message Type
{
    DisplayTask_IPMCT_01_ShowEncoderTaskInitializationStatus,
    DisplayTask_IPCMT_02_ShowControllerRXTaskInitializationStatus,
    DisplayTask_IPMCT_03_ShowControllerTXTaskInitializationStatus,
    DisplayTask_IPMCT_04_ShowBatteryTaskInitializationStatus,

    DisplayTask_IPMCT_05_ShowEncoderReps,
    DisplayTask_IPCMT_06_ShowControllerTaskConnectionStatus,
    DisplayTask_IPCMT_07_ShowBatteryLevel,
    DisplayTask_IPCMT_08_ShowChargingStatus
} DisplayTask_IPC_Message_Type;

typedef struct
{
    // IPCMT: Inter Process Communication Message Type
    DisplayTask_IPC_Message_Type displayTask_IPCMT;
    union
    {
        uint8_t _01_isEncoderTaskInitialized;
        uint8_t _02_isControllerRXTaskInitialized;
        uint8_t _03_isControllerTXTaskInitialized;
        uint8_t _04_isBatteryTaskInitialized;

        uint16_t _05_numberOfReps;
        uint8_t _06_isConnected;
        uint16_t _07_batteryLevel;
        uint8_t _08_isCharging;
    } payload;
} DisplayTask_IPC_Message_Struct;

void __IPC_displayTask_SendMessage_ShowEncoderTaskIntizializationStatus(uint8_t isEncoderTaskInitialized);
void __IPC_displayTask_SendMessage_ShowControllerRXTaskInitializationStatus(uint8_t isControllerRXTaskInitialized);
void __IPC_displayTask_SendMessage_ShowControllerTXTaskInitializationStatus(uint8_t isControllerTXTaskInitialized);
void __IPC_displayTask_SendMessage_ShowBatteryTaskInitializationStatus(uint8_t isBatteryTaskInitialized);

void __IPC_displayTask_SendMessage_ShowEncoderReps(uint16_t numberOfReps);
void __IPC_displayTask_SendMessage_ShowControllerTaskConnectionStatus(uint8_t isConnected);
void __IPC_displayTask_SendMessage_ShowBatteryLevel(uint16_t batteryLevel);
void __IPC_displayTask_SendMessage_ShowChargingStatus(uint8_t isCharging);