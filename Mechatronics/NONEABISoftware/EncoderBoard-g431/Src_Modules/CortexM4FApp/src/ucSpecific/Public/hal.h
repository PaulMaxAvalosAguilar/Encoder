#ifndef HAL_H
#define HAL_H

#include <inttypes.h>

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

//-------------------SysFrequency-----------------------------------------------------------------
void __ucHAL_SysFreq_configure_initialState(void);
void __ucHAL_SysFreq_function_enterSleepState(void);
void __ucHAL_SysFreq_function_exitSleepState(void);

//-------------------DMA-------------------------------------------------------------------
void __ucHAL_DMA_configure(void);

//-------------------RTOSTIMER-------------------------------------------------------------
void __ucHAL_RtosTimer_function_pause(void);
void __ucHAL_RtosTimer_function_resume(void);

//-------------------DISPLAY---------------------------------------------------------------
void __ucHAL_Display_configure(void);
void __ucHAL_Display_function_transmit(
    uint8_t i2c_addr, uint8_t *buffer, uint32_t nbytes);

//-------------------POWERSYSTEM-------------------------------------------------------------------
void __ucHAL_POWERSYSTEM_configure(void);

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