#include "displayTask.h"

// RTOS
#include "task.h"
#include "queue.h"

// Processes
#include "Processes/Priority1/batteryTask.h"
#include "Processes/Priority2/controllerTXTask.h"
#include "Processes/Priority3/controllerRXTask.h"
#include "Processes/Priority4/encoderTask.h"

// Utilities
#include "Utilities/itoa.h"

// StdLib
#include <string.h>
#include <stdlib.h>

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

    const unsigned int ScreenPosition_Header = 0;
    const unsigned int ScreenPosition_TasksInitialization = 2;
    const unsigned int ScreenPosition_EncoderReps = 3;
    const unsigned int ScreenPosition_ControllerTaskConnectionStatus = 4;
    const unsigned int ScreenPosition_BatteryLevelAndChargingStatus = 5;

    const char batteryNotChargingMessage[] = "   Battery:";
    const char batteryChargingMessage[] = "    **Battery:";

    uint16_t batteryLevel = 0;
    uint8_t batteryIsCharging = 0;

    // Show that task inizialization worked
    lcdPutsBlinkFreeAtPos("  Paul's Inventions   ", ScreenPosition_Header);

    // Pre loop tasks
    vTaskSuspendAll();
    __controllerTXTask_init();
    __batteryTask_init();
    __controllerRXTask_init();
    __encoderTask_init();
    xTaskResumeAll();

    for (;;)
    {
        xQueueReceive(displayTask_QueueHandle, &rMessage, portMAX_DELAY);

        switch (rMessage.displayTask_IPCMT) // Check received message type
        {
        case DisplayTask_IPMCT_01_ShowEncoderTaskInitializationStatus:
            lcdPutIntegerAtPos(4, 4, ScreenPosition_TasksInitialization);
            break;
        case DisplayTask_IPCMT_02_ShowControllerRXTaskInitializationStatus:
            lcdPutIntegerAtPos(3, 3, ScreenPosition_TasksInitialization);
            break;
        case DisplayTask_IPMCT_03_ShowControllerTXTaskInitializationStatus:
            lcdPutIntegerAtPos(2, 2, ScreenPosition_TasksInitialization);
            break;
        case DisplayTask_IPMCT_04_ShowBatteryTaskInitializationStatus:
            lcdPutIntegerAtPos(1, 1, ScreenPosition_TasksInitialization);
            break;

        case DisplayTask_IPMCT_05_ShowEncoderReps:
            lcdPutIntegerAtPos(rMessage.payload._05_numberOfReps, 1, ScreenPosition_EncoderReps);
            break;
        case DisplayTask_IPCMT_06_ShowControllerTaskConnectionStatus:
            break;
        case DisplayTask_IPCMT_07_ShowBatteryLevel:
        case DisplayTask_IPCMT_08_ShowChargingStatus:
            if (rMessage.displayTask_IPCMT == DisplayTask_IPCMT_07_ShowBatteryLevel)
            {
                batteryLevel = rMessage.payload._07_batteryLevel;
            }
            else
            {
                batteryIsCharging = rMessage.payload._08_isCharging;
            }

            lcdPutStringAndIntBlinksFreeAtPos(batteryIsCharging ? batteryChargingMessage
                                                                : batteryNotChargingMessage,
                                              batteryLevel,
                                              ScreenPosition_BatteryLevelAndChargingStatus);
            break;
        }
    }
}

void __IPC_displayTask_SendMessage_ShowEncoderTaskIntizializationStatus(uint8_t isEncoderTaskInitialized)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPMCT_01_ShowEncoderTaskInitializationStatus;
    message.payload._01_isEncoderTaskInitialized = isEncoderTaskInitialized;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}

void __IPC_displayTask_SendMessage_ShowControllerRXTaskInitializationStatus(uint8_t isControllerRXTaskInitialized)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPCMT_02_ShowControllerRXTaskInitializationStatus;
    message.payload._02_isControllerRXTaskInitialized = isControllerRXTaskInitialized;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}

void __IPC_displayTask_SendMessage_ShowControllerTXTaskInitializationStatus(uint8_t isControllerTXTaskInitialized)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPMCT_03_ShowControllerTXTaskInitializationStatus;
    message.payload._03_isControllerTXTaskInitialized = isControllerTXTaskInitialized;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}

void __IPC_displayTask_SendMessage_ShowBatteryTaskInitializationStatus(uint8_t isBatteryTaskInitialized)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPMCT_04_ShowBatteryTaskInitializationStatus;
    message.payload._04_isBatteryTaskInitialized = isBatteryTaskInitialized;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}

void __IPC_displayTask_SendMessage_ShowEncoderReps(uint16_t numberOfReps)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPMCT_05_ShowEncoderReps;
    message.payload._05_numberOfReps = numberOfReps;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}

void __IPC_displayTask_SendMessage_ShowControllerTaskConnectionStatus(uint8_t isConnected)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPCMT_06_ShowControllerTaskConnectionStatus;
    message.payload._06_isConnected = isConnected;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}

void __IPC_displayTask_SendMessage_ShowBatteryLevel(uint16_t batteryLevel)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPCMT_07_ShowBatteryLevel;
    message.payload._07_batteryLevel = batteryLevel;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}

void __IPC_displayTask_SendMessage_ShowChargingStatus(uint8_t isCharging)
{
    static DisplayTask_IPC_Message_Struct message;
    message.displayTask_IPCMT = DisplayTask_IPCMT_08_ShowChargingStatus;
    message.payload._08_isCharging = isCharging;
    xQueueSendToBack(displayTask_QueueHandle, &message, portMAX_DELAY);
}