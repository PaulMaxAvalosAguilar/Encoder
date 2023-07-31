#include "FreeRTOS.h"
#include "task.h"

#pragma once

// Task Parameters-----------------------------------------------

#define ENCODER_TASK_STACK_SIZE 128
#define ENCODER_TASK_STACK_PRIORITY 3

// Task Manipulation---------------------------------------------
void __encoderTask_init(void);

// IPC: Inter Process Comumnication------------------------------
#define ENCODER_TASK_QUEUE_SIZE 20

typedef enum // IPCMT: Inter Process Communication Message Type
{
    EncoderTask_IPCMT_01_StartMeasuring
} EncoderTask_IPC_Message_Type;

typedef struct
{
    uint32_t minDistToTravel;
    uint32_t desiredCounterDirection;
    uint32_t desiredRepDirection;
} measuringParameters;

typedef struct
{
    // IPCMT: Inter Process Communication Message Type
    EncoderTask_IPC_Message_Type batteryTask_IPCMT;
    union
    {
        measuringParameters _01_StartMeasuring;

    } payload;
} EncoderTask_IPC_Message_Struct;

void __IPC_encoderTask_SendMessage_StartMeasuring(uint32_t minDistToTravel,
                                                  uint32_t desiredCounterDirection,
                                                  uint32_t desiredRepDirection);
