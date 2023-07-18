#include "FreeRTOS.h"

#pragma once

// Task Parameters-----------------------------------------------

#define BATTERY_TASK_STACK_SIZE 128
#define BATTERY_TASK_STACK_PRIORITY 1
#define DISCHARGING_TIMER_PERIOD pdMS_TO_TICKS(1500)
#define CHARGING_TIMER_PERIOD pdMS_TO_TICKS(500)

// Task Manipulation---------------------------------------------
void __batteryTask_init(void);
void __batteryTask_pause(void);
void __batteryTask_resume(void);
void __batteryTask_enterChargingRate(void);
void __batteryTask_enterDischargingRate(void);

// IPC: Inter Process Comumnication------------------------------
#define BATTERY_TASK_QUEUE_SIZE 20

typedef enum // IPCMT: Inter Process Communication Message Type
{
    BatteryTask_IPCMT_01_PerformBatteryAdcConversion
} BatteryTask_IPC_Message_Type;

typedef struct
{
    // IPCMT: Inter Process Communication Message Type
    BatteryTask_IPC_Message_Type batteryTask_IPCMT;
    union
    {
        uint8_t _01_shouldPerformConversion;

    } payload;
} BatteryTask_IPC_Message_Struct;

void __IPC_batteryTask_SendMessage_PerformBatteryAdcConversion(uint8_t _01_shouldPerformConversion);