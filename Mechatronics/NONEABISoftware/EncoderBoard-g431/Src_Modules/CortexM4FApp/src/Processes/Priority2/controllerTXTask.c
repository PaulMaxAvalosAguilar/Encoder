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

    // Module inizialization

    // Internal Variable Inizialization
    ControllerTXTask_IPC_Message_Struct rMessage; // receivedMessage

    // Show that task inizialization worked
    __IPC_displayTask_SendMessage_ShowControllerTXTaskInitializationStatus(1);

    for (;;)
    {
        xQueueReceive(controllerTXTask_QueueHandle, &rMessage, portMAX_DELAY);

        switch (rMessage.batteryTask_IPCMT) // Check received message type
        {
        case ControllerTXTask_IPMCT_01_SendEncoderData:
            break;
        case ControllerTXTask_IPMCT_02_ConfirmEncoderTaskStarted:
            break;
        case ControllerTXTask_IPMCT_03_ConfirmEncoderTaskStoped:
            break;
        case ControllerTXTask_IPMCT_04_SendBatteryLevel:
            break;
        case ControllerTXTask_IPMCT_05_SendChargingStatus:
            break;
        }
    }
}