#include "controllerRXTask.h"

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

TaskHandle_t controllerRXTask_Handle = NULL;
QueueHandle_t controllerRXTask_QueueHandle = NULL;

void controllerRXTask(void *args);

void __controllerRXTask_init(void)
{
    xTaskCreate(controllerRXTask,
                "controllerRXTask",
                CONTROLLERRX_TASK_STACK_SIZE,
                NULL,
                CONTROLLERRX_TASK_STACK_PRIORITY,
                &controllerRXTask_Handle);

    controllerRXTask_QueueHandle = xQueueCreate(CONTROLLERRX_TASK_QUEUE_SIZE, sizeof(ControllerRXTask_IPC_Message_Struct));
}

void controllerRXTask(void *args __attribute__((unused)))
{
    // HAL inizialization
    __ucHAL_Bluetooth_configure();

    // Module inizialization

    // Internal Variable Inizialization
    ControllerRXTask_IPC_Message_Struct rMessage; // receivedMessage

    // Show that task inizialization worked
    __IPC_displayTask_SendMessage_ShowControllerRXTaskInitializationStatus(1);

    for (;;)
    {
        xQueueReceive(controllerRXTask_QueueHandle, &rMessage, portMAX_DELAY);
    }
}