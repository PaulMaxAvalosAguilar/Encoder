#include "controllerTXTask.h"

// RTOS
#include "task.h"
#include "queue.h"
#include "timers.h"

// Processes
#include "batteryTask.h"
#include "displayTask.h"

// Utilities

// StdLib

// uCSpecific

// Used modules

TaskHandle_t controllerTask_Handle = NULL;
QueueHandle_t controllerTask_QueueHandle = NULL;

void controllerTask(void *args);

void __controllerTask_init(void)
{
    xTaskCreate(controllerTask,
                "controllerTask",
                CONTROLLER_TASK_STACK_SIZE,
                NULL,
                CONTROLLER_TASK_STACK_PRIORITY,
                &controllerTask_Handle);

    controllerTask_QueueHandle = xQueueCreate(CONTROLLER_TASK_QUEUE_SIZE, sizeof(ControllerTask_IPC_Message_Struct));
}

void controllerTask(void *args __attribute__((unused)))
{
    // HAL inizialization

    // Module inizialization

    // Internal Variable Inizialization

    // Show that task inizialization worked

    for (;;)
    {
    }
}