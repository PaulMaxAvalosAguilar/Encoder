#ifndef HAL_H
#define HAL_H

#include <inttypes.h>
#include "Utilities/RING/ring.h"

/*
This file expose the functions an encoder should need to work. They hide the internal
implementation of any specific MCU type. **DMA is optional

_configure and _function function types should clock gate if available and feasable.
*/

//-------------------INTERRUPTS------------------------------------------------------------
void __ucHAL_Interrupts_configure(void);
void __ucHAL_Interrupts_function_disableInterrupts(void);
void __ucHAL_Interrupts_function_reenableInterrupts(void);
void __ucHAL_Interrupts_function_WaitForInterrupt(void);

//-------------------SYSFREQUENCY----------------------------------------------------------
void __ucHAL_SysFreq_configure_initialState(void);
void __ucHAL_SysFreq_function_enterSleepState(void);
void __ucHAL_SysFreq_function_exitSleepState(void);

//-------------------DMA-------------------------------------------------------------------
void __ucHAL_DMA_configure(void);

//-------------------RTOSTIMER-------------------------------------------------------------
void __ucHAL_RtosTimer_function_pause(void);
void __ucHAL_RtosTimer_function_resume(void);

//-------------------ENCODER---------------------------------------------------------------
#define ENCODER_BUFFER_SIZE 256
extern ring_t encoder_ring;
typedef struct encoderValues_t
{
    uint16_t encoderCounter;
    uint32_t inputCapture;
} encoderValues_t;
extern encoderValues_t encoderBuffer[ENCODER_BUFFER_SIZE];
void __ucHAL_Encoder_configure(void);

void __ucHAL_Encoder_function_ITAddEncoderValues(void);
int __ucHAL_Encoder_function_ITReadEncoderValues(void *data);

//-------------------BLUETOOTH-------------------------------------------------------------
#define BLUETOOTH_RX_BUFFER_LEN 256
extern uint8_t bluetoothReceiveBuffer[BLUETOOTH_RX_BUFFER_LEN];

void __ucHAL_Bluetooth_configure(void);
void __ucHAL_Bluetooth_function_transmit(uint8_t *buffer, uint32_t nbytes);

//-------------------DISPLAY---------------------------------------------------------------
void __ucHAL_Display_configure(void);
void __ucHAL_Display_function_transmit(
    uint8_t i2c_addr, uint8_t *buffer, uint32_t nbytes);

//-------------------BATTERY---------------------------------------------------------------
void __ucHAL_Battery_configure(void);

//-------------------SLEEPTIMER------------------------------------------------------------
typedef struct SleepTimerValues
{
    uint16_t counter;
    uint8_t updateInterruptOcurred;
} SleepTimerValues;

void __ucHAL_Sleeptimer_configure(void);
void __ucHAL_Sleeptimer_startOneShot(uint16_t initialValue, uint16_t autoReloadValue);
SleepTimerValues __ucHAL_Sleeptimer_stopOneShot(void);

#endif