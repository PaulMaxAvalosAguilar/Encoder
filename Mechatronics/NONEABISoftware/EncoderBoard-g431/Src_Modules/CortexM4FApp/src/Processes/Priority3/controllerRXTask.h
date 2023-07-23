#include "FreeRTOS.h"

#pragma once

// Task Parameters-----------------------------------------------

#define CONTROLLERRX_TASK_STACK_SIZE 128
#define CONTROLLERRX_TASK_STACK_PRIORITY 3

// Task Manipulation---------------------------------------------
void __controllerRXTask_init(void);

// IPC: Inter Process Comumnication------------------------------
#define CONTROLLERRX_TASK_QUEUE_SIZE 20

typedef enum // IPCMT: Inter Process Communication Message Type
{
    ControllerRXTask_IPCMT_01_PerformBatteryAdcConversion
} ControllerRXTask_IPC_Message_Type;

typedef struct
{
    // IPCMT: Inter Process Communication Message Type
    ControllerRXTask_IPC_Message_Type batteryTask_IPCMT;
    union
    {
        uint8_t _01_shouldPerformConversion;

    } payload;
} ControllerRXTask_IPC_Message_Struct;