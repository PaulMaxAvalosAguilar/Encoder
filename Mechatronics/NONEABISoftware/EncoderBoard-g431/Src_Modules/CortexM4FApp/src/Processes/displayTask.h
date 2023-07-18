#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#pragma once

// Task Parameters-----------------------------------------------

#define DISPLAY_TASK_STACK_SIZE 256
#define DISPLAY_TASK_PRIORITY 2

extern TaskHandle_t displayTask_Handle;

void __displayTask_init(void);

// IPC: Inter Process Comumnication------------------------------
#define DISPLAY_TASK_QUEUE_SIZE 20
extern QueueHandle_t displayTask_QueueHandle;

typedef enum // IPCMT: Inter Process Communication Message Type
{
    DisplayTask_IPCMT_01_ShowInitialConfigurationStatus,
    DisplayTask_IPCMT_02_ShowReadyForConnectionStatus,
    DisplayTask_IPCMT_03_ShowControllerTaskConnectionStatus,
    DisplayTask_IPCMT_04_ShowBatteryLevel,
    DisplayTask_IPCMT_05_ShowChargingStatus
} DisplayTask_IPC_Message_Type;

typedef struct
{
    // IPCMT: Inter Process Communication Message Type
    DisplayTask_IPC_Message_Type displayTask_IPCMT;
    union
    {
        uint8_t _01_isConfiguredForInitialWorking;
        uint8_t _02_isReadyForConnection;
        uint8_t _03_isConnected;
        uint16_t _04_batteryLevel;
        uint8_t _05_isCharging;
    } payload;
} DisplayTask_IPC_Message_Struct;

void __IPC_displayTask_SendMessage_ShowInitialConfigurationStatus(uint8_t isConfiguredForInitialWorking);
void __IPC_displayTask_SendMessage_ShowReadyForConnectionStatus(uint8_t isReadyForConnection);
void __IPC_displayTask_SendMessage_ShowControllerTaskConnectionStatus(uint8_t isConnected);
void __IPC_displayTask_SendMessage_ShowBatteryLevel(uint16_t batteryLevel);
void __IPC_displayTask_SendMessage_ShowChargingStatus(uint8_t isCharging);
