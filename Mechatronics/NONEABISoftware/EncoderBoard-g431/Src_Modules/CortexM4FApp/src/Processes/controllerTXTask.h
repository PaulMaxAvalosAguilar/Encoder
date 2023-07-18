#include "FreeRTOS.h"

#pragma once

// Task Parameters-----------------------------------------------

#define CONTROLLER_TXTASK_STACK_SIZE 128
#define CONTROLLER_TXTASK_STACK_PRIORITY 1

// Task Manipulation---------------------------------------------
void __controllerTask_init(void);

// IPC: Inter Process Comumnication------------------------------
#define CONTROLLER_TASK_QUEUE_SIZE 20

typedef enum // IPCMT: Inter Process Communication Message Type
{
    ControllerTask_IPCMT_01_PerformBatteryAdcConversion
} ControllerTask_IPC_Message_Type;

typedef struct
{
    // IPCMT: Inter Process Communication Message Type
    ControllerTask_IPC_Message_Type batteryTask_IPCMT;
    union
    {
        uint8_t _01_shouldPerformConversion;

    } payload;
} ControllerTask_IPC_Message_Struct;