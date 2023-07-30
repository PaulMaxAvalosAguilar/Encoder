#include "controllerTXTask.h"

// RTOS
#include "task.h"
#include "queue.h"
#include "timers.h"

// Processes
#include "Processes/Priority2/displayTask.h"

// Utilities

// StdLib

// uCSpecific

// Used modules
#include "Modules/Bluetooth/Public/bluetooth.h"

TaskHandle_t controllerTXTask_Handle = NULL;
QueueHandle_t controllerTXTask_QueueHandle = NULL;

void controllerTXTask(void *args);

void __controllerTXTask_init(void)
{
    xTaskCreate(controllerTXTask,
                "controllerTXTask",
                CONTROLLERTX_TASK_STACK_SIZE,
                NULL,
                CONTROLLERTX_TASK_STACK_PRIORITY,
                &controllerTXTask_Handle);

    controllerTXTask_QueueHandle = xQueueCreate(CONTROLLERTX_TASK_QUEUE_SIZE, sizeof(ControllerTXTask_IPC_Message_Struct));
}

void controllerTXTask(void *args __attribute__((unused)))
{
    // HAL inizialization
    //__ucHAL_Bluetooth_configure();
    // Module inizialization

    // Internal Variable Inizialization
    ControllerTXTask_IPC_Message_Struct rMessage; // receivedMessage
    uint16_t batteryLevel = 0;

    // Show that task inizialization worked
    __IPC_displayTask_SendMessage_ShowControllerTXTaskInitializationStatus(1);

    for (;;)
    {
        xQueueReceive(controllerTXTask_QueueHandle, &rMessage, portMAX_DELAY);

        switch (rMessage.batteryTask_IPCMT) // Check received message type
        {
        case ControllerTXTask_IPMCT_01_SendEncoderData:
            bluetoothSendEncoderData(rMessage.payload._01_encoderData.rom,
                                     rMessage.payload._01_encoderData.meanPropVelocity,
                                     rMessage.payload._01_encoderData.peakVelocity);
            break;
        case ControllerTXTask_IPMCT_02_ControllerConnectionStatus:
            bluetoothSendIsControllerConnected(rMessage.payload._02_isControllerConnected);
            break;
        case ControllerTXTask_IPMCT_03_ConfirmEncoderTaskStarted:
            bluetoothSendIsEncoderTaskStarted(rMessage.payload._03_isEncoderTaskStarted);
            break;
        case ControllerTXTask_IPMCT_04_ConfirmEncoderTaskStoped:
            bluetoothSendIsEncoderTaskStopped(rMessage.payload._04_isEncoderTaskStopped);
            break;
        case ControllerTXTask_IPMCT_05_SendBatteryLevel:
            batteryLevel = rMessage.payload._05_batteryLevel;
            // bluetoothSendBatteryLevel(batteryLevel);
            break;
        case ControllerTXTask_IPMCT_06_SendChargingStatus:
            bluetoothSendChargingStatus(rMessage.payload._06_isEncoderCharging);
            break;
        }
    }
}

void __IPC_controllerTXTask_SendMessage_SendEncoderData(uint16_t rom, uint16_t meanPropVelocity, uint16_t peakVelocity)
{
    ControllerTXTask_IPC_Message_Struct message;
    message.batteryTask_IPCMT = ControllerTXTask_IPMCT_01_SendEncoderData;
    message.payload._01_encoderData.rom = rom;
    message.payload._01_encoderData.meanPropVelocity = meanPropVelocity;
    message.payload._01_encoderData.peakVelocity = peakVelocity;
    xQueueSendToBack(controllerTXTask_QueueHandle, &message, 0);
}

void __IPC_controllerTXTask_SendMessage_ControllerConnectionStatus(uint8_t isControllerConected)
{
    ControllerTXTask_IPC_Message_Struct message;
    message.batteryTask_IPCMT = ControllerTXTask_IPMCT_02_ControllerConnectionStatus;
    message.payload._02_isControllerConnected = isControllerConected;
    xQueueSendToBack(controllerTXTask_QueueHandle, &message, 0);
}

void __IPC_controllerTXTask_SendMessage_ConfirmEncoderTaskStarted(uint8_t isEncoderTaskStarted)
{
    ControllerTXTask_IPC_Message_Struct message;
    message.batteryTask_IPCMT = ControllerTXTask_IPMCT_03_ConfirmEncoderTaskStarted;
    message.payload._03_isEncoderTaskStarted = isEncoderTaskStarted;
    xQueueSendToBack(controllerTXTask_QueueHandle, &message, 0);
}

void __IPC_controllerTXTask_SendMessage_ConfirmEncoderTaskStoped(uint8_t isEncoderTaskStopped)
{
    ControllerTXTask_IPC_Message_Struct message;
    message.batteryTask_IPCMT = ControllerTXTask_IPMCT_04_ConfirmEncoderTaskStoped;
    message.payload._04_isEncoderTaskStopped = isEncoderTaskStopped;
    xQueueSendToBack(controllerTXTask_QueueHandle, &message, 0);
}

void __IPC_controllerTXTask_SendMessage_SendBatteryLevel(uint16_t batteryLevel)
{
    ControllerTXTask_IPC_Message_Struct message;
    message.batteryTask_IPCMT = ControllerTXTask_IPMCT_05_SendBatteryLevel;
    message.payload._05_batteryLevel = batteryLevel;
    xQueueSendToBack(controllerTXTask_QueueHandle, &message, 0);
}

void __IPC_controllerTXTask_SendMessage_SendChargingStatus(uint8_t isEncoderCharging)
{
    ControllerTXTask_IPC_Message_Struct message;
    message.batteryTask_IPCMT = ControllerTXTask_IPMCT_06_SendChargingStatus;
    message.payload._06_isEncoderCharging = isEncoderCharging;
    xQueueSendToBack(controllerTXTask_QueueHandle, &message, 0);
}