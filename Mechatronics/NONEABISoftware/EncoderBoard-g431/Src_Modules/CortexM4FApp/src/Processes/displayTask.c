#include "displayTask.h"

// RTOS
#include "task.h"
#include "queue.h"

// Processes
#include "controllerTask.h"
#include "batteryTask.h"

// Utilities
#include "Utilities/itoa.h"

// StdLib
#include <string.h>

// uCSpecific
#include "ucSpecific/Public/hal.h"
// Used modules
#include "Modules/LCD/Public/lcd.h"

TaskHandle_t displayTask_Handle = NULL;
QueueHandle_t displayTask_QueueHandle = NULL;

void displayTask(void *args);

void __displayTask_init()
{
    displayTask_QueueHandle = xQueueCreate(DISPLAY_TASK_QUEUE_SIZE, sizeof(DisplayTask_IPC_Message_Struct));

    xTaskCreate(
        displayTask,
        "displayTask",
        DISPLAY_TASK_STACK_SIZE,
        NULL,
        DISPLAY_TASK_PRIORITY,
        &displayTask_Handle);
}

void displayTask(void *args __attribute__((unused)))
{
    // HAL inizialization
    __ucHAL_Display_configure();

    // Module inizialization
    lcd_init();

    // Iternal variable inizalization
    DisplayTask_IPC_Message_Struct rMessage; // received Message
    unsigned int number = 10;
    char buffer[22];
    char numBuffer[5];

    // Show that task inizialization worked
    lcdPutsBlinkFree("  Paul's Inventions   ", 0);

    // Configure child tasks
    vTaskSuspendAll();
    //__controllerTask_init();
    __batteryTask_init();
    xTaskResumeAll();

    for (;;)
    {
        xQueueReceive(displayTask_QueueHandle, &rMessage, portMAX_DELAY);

        switch (rMessage.displayTask_IPCMT)
        {
        case DisplayTask_IPCMT_04_ShowBatteryLevel:
            strcpy(buffer, "HOLA: ");
            // itoa(number++, numBuffer, 10);
            itoa(rMessage.payload._04_batteryLevel, numBuffer, 10);
            strcat(buffer, numBuffer);
            lcdPutsBlinkFree(buffer, 5);
            break;
        }
    }
}

void __IPC_displayTask_SendMessage_ShowInitialConfigurationStatus(uint8_t isConfiguredForInitialWorking)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPCMT_01_ShowInitialConfigurationStatus;
    message.payload._01_isConfiguredForInitialWorking = isConfiguredForInitialWorking;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}
void __IPC_displayTask_SendMessage_ShowReadyForConnectionStatus(uint8_t isReadyForConnection)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPCMT_02_ShowReadyForConnectionStatus;
    message.payload._02_isReadyForConnection = isReadyForConnection;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}
void __IPC_displayTask_SendMessage_ShowControllerTaskConnectionStatus(uint8_t isConnected)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPCMT_03_ShowControllerTaskConnectionStatus;
    message.payload._03_isConnected = isConnected;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}
void __IPC_displayTask_SendMessage_ShowBatteryLevel(uint16_t batteryLevel)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPCMT_04_ShowBatteryLevel;
    message.payload._04_batteryLevel = batteryLevel;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}
void __IPC_displayTask_SendMessage_ShowChargingStatus(uint8_t isCharging)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPCMT_05_ShowChargingStatus;
    message.payload._05_isCharging = isCharging;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}