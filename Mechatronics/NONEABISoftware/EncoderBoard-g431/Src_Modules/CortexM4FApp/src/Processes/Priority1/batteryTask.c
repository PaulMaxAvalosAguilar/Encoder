#include "batteryTask.h"

// RTOS
#include "task.h"
#include "queue.h"
#include "timers.h"

// Processes
#include "Processes/Priority2/displayTask.h"
#include "Processes/Priority2/controllerTXTask.h"

// Utilities

// StdLib

// uCSpecific
#include "ucSpecific/Public/hal.h"
// Used modules

TaskHandle_t batteryTask_Handle = NULL;
TimerHandle_t batteryTaskTimer_Handle = NULL;
QueueHandle_t batteryTask_QueueHandle = NULL;

void batteryTask(void *args);
void timerCallback(TimerHandle_t xTimer);

void __batteryTask_init(void)
{
    xTaskCreate(batteryTask,
                "batteryTask",
                BATTERY_TASK_STACK_SIZE,
                NULL,
                BATTERY_TASK_STACK_PRIORITY,
                &batteryTask_Handle);

    batteryTask_QueueHandle = xQueueCreate(BATTERY_TASK_QUEUE_SIZE, sizeof(BatteryTask_IPC_Message_Struct));

    batteryTaskTimer_Handle = xTimerCreate("batteryTimer", DISCHARGING_TIMER_PERIOD, pdTRUE, (void *)0, timerCallback);
    xTimerStart(batteryTaskTimer_Handle, portMAX_DELAY);
}

void __batteryTask_pause()
{
    xTimerStop(batteryTaskTimer_Handle, portMAX_DELAY);
    vTaskSuspend(batteryTask_Handle);
}

void __batteryTask_resume()
{
    vTaskResume(batteryTask_Handle);
    xTimerReset(batteryTaskTimer_Handle, portMAX_DELAY);
}

void __batteryTask_enterChargingRate(void)
{
    xTimerChangePeriod(batteryTaskTimer_Handle, CHARGING_TIMER_PERIOD, portMAX_DELAY);
}

void __batteryTask_enterDischargingRate(void)
{
    xTimerChangePeriod(batteryTaskTimer_Handle, DISCHARGING_TIMER_PERIOD, portMAX_DELAY);
}

void batteryTask(void *args __attribute__((unused)))
{
    // HAL inizialization
    __ucHAL_Battery_configure();

    // Module inizialization

    // Internal Variable Inizialization
    BatteryTask_IPC_Message_Struct rMessage; // received Message
    int i = 10;
    // Show that task inizialization worked
    __IPC_displayTask_SendMessage_ShowBatteryTaskInitializationStatus(1);

    // Pre loop tasks
    __IPC_displayTask_SendMessage_ShowBatteryLevel(i);
    __IPC_controllerTXTask_SendMessage_SendBatteryLevel(i);

    for (;;)
    {
        xQueueReceive(batteryTask_QueueHandle, &rMessage, portMAX_DELAY);

        __IPC_displayTask_SendMessage_ShowBatteryLevel(i++);
        __IPC_controllerTXTask_SendMessage_SendBatteryLevel(i);
    }
}

void timerCallback(TimerHandle_t xTimer __attribute__((unused)))
{
    __IPC_batteryTask_SendMessage_PerformBatteryAdcConversion(1);
}

void __IPC_batteryTask_SendMessage_PerformBatteryAdcConversion(uint8_t _01_shouldPerformConversion)
{
    static BatteryTask_IPC_Message_Struct message;
    message.batteryTask_IPCMT = BatteryTask_IPCMT_01_PerformBatteryAdcConversion;
    message.payload._01_shouldPerformConversion = _01_shouldPerformConversion;
    xQueueSendToBack(batteryTask_QueueHandle, &message, portMAX_DELAY);
}