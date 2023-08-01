#include "encoderTask.h"

#include "queue.h"
#include "task.h"

#include "ucSpecific/Public/hal.h"

#include "Processes/Priority2/displayTask.h"

TaskHandle_t encoderTask_Handle = NULL;
QueueHandle_t encoderTask_QueueHandle = NULL;

void encoderTask(void *args);

void __encoderTask_init()
{
    xTaskCreate(
        encoderTask,
        "encoderTask",
        ENCODER_TASK_STACK_SIZE,
        NULL,
        ENCODER_TASK_STACK_PRIORITY,
        &encoderTask_Handle);

    encoderTask_QueueHandle = xQueueCreate(ENCODER_TASK_QUEUE_SIZE, sizeof(EncoderTask_IPC_Message_Struct));
}

void encoderTask(void *args __attribute((unused)))
{
    // HAL inizialization
    __ucHAL_Encoder_configure();
    // Module inizialization

    // Internal Variable Inizialization
    EncoderTask_IPC_Message_Struct rMessage; // receivedMessage
    encoderValues_t receivedEncoderValues;

    // Show that task inizialization worked
    __IPC_displayTask_SendMessage_ShowEncoderTaskIntizializationStatus(1);
    __IPC_displayTask_SendMessage_ShowEncoderReps((uint16_t)receivedEncoderValues.encoderCounter);

    for (;;)
    {
        // xQueueReceive(encoderTask_QueueHandle, &rMessage, portMAX_DELAY);

        while (__ucHAL_Encoder_function_ITReadEncoderValues(&receivedEncoderValues) != -1)
        {
            static int i = 0;
            receivedEncoderValues.encoderCounter;
            receivedEncoderValues.inputCapture;

            __IPC_displayTask_SendMessage_ShowEncoderReps(++i);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void __IPC_encoderTask_SendMessage_StartMeasuring(uint32_t minDistToTravel,
                                                  uint32_t desiredCounterDirection,
                                                  uint32_t desiredRepDirection)
{
    EncoderTask_IPC_Message_Struct message;
    message.encoderTask_IPCMT = EncoderTask_IPCMT_01_StartMeasuring;
    message.payload._01_StartMeasuring.minDistToTravel = minDistToTravel;
    message.payload._01_StartMeasuring.desiredCounterDirection = desiredCounterDirection;
    message.payload._01_StartMeasuring.desiredRepDirection = desiredRepDirection;
    xQueueSendToBack(encoderTask_QueueHandle, &message, 0);
}