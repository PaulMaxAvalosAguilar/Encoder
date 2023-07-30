#include "hal.h"
#include "../Internal/dma.h"
#include "../Internal/drivers.h"
#include "stm32g431xx.h"

//-------------------INTERRUPTS------------------------------------------------------------
void __ucHAL_Interrupts_configure()
{
    NVIC_SetPriorityGrouping(0); // 4 bits for pre-emption 0 bit for subpriority

    NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_SetPriority(LPUART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
    NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));

    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(LPUART1_IRQn);
}

void __ucHAL_Interrupts_function_disableInterrupts()
{
    // Disable interrupts
    __asm volatile("cpsid i" ::
                       : "memory");
    __asm volatile("dsb");
    __asm volatile("isb");
}

void __ucHAL_Interrupts_function_reenableInterrupts()
{
    __asm volatile("cpsie i" ::
                       : "memory");
    __asm volatile("dsb");
    __asm volatile("isb");
}

void __ucHAL_Interrupts_function_WaitForInterrupt()
{
    __asm volatile("dsb" ::
                       : "memory");
    __asm volatile("wfi");
    __asm volatile("isb");
}

//-------------------SYSFREQUENCY-------------------------------------------------------------
void __ucHAL_SysFreq_configure_initialState()
{
    __ucDrivers_FLASH_function_EnableCache();
    __ucDrivers_FLASH_function_Set1WSLatency();

    __ucDrivers_RCC_function_TurnHSEOn();
    __ucDrivers_RCC_function_SetHSEAsPLLSource();
    __ucDrivers_RCC_function_SetPLLmn50MhzOn8MhzHSE();
    __ucDrivers_RCC_function_TurnPLLOn();
    __ucDrivers_RCC_function_EnablePLLROutput();
    __ucDrivers_RCC_function_SelectPLLAsClockSource();
}

void __ucHAL_SysFreq_function_enterSleepState()
{
    __ucDrivers_RCC_function_AHBPrescale97656KhzFrom50mhz();
    __ucDrivers_PWR_enable_Clock();
    __ucDrivers_PWR_function_TurnOnLPR();
}

void __ucHAL_SysFreq_function_exitSleepState()
{
    __ucDrivers_PWR_function_TurnOffLPR();
    __ucDrivers_RCC_function_AHBPrescalerReset();
    __ucDrivers_PWR_disable_Clock();
}

//-------------------DMA-------------------------------------------------------------------
void __ucHAL_DMA_configure()
{
    __ucDrivers_DMA_enable_Clock();
    __ucDrivers_DMAMUX_enable_Clock();
    __ucDrivers_DMA_conf_Channel1ForLPUART_RX(bluetoothReceiveBuffer, BLUETOOTH_RX_BUFFER_LEN);
    __ucDrivers_DMA_conf_Channel2ForLPUART_TX();
    __ucDrivers_DMA_conf_Channel3ForI2C_TX();
}

//-------------------RTOSTIMER-------------------------------------------------------------
void __ucHAL_RtosTimer_function_pause()
{
    __ucDrivers_SYSTICK_function_disableTimer();
}

void __ucHAL_RtosTimer_function_resume()
{
    __ucDrivers_SYSTICK_function_enableTimer();
}

//-------------------BLUETOOH---------------------------------------------------------------
uint8_t bluetoothReceiveBuffer[BLUETOOTH_RX_BUFFER_LEN] = {0};

void __ucHAL_Bluetooth_configure()
{
    __ucDrivers_LPUART_conf_IndependentClock_SysClkSource();
    __ucDrivers_LPUART_enable_Clock();
    __ucDrivers_LPUART_conf_Periphereal();

    __ucDrivers_LPUART_enable_GPIO_Clock();
    __ucDrivers_LPUART_conf_GPIO_Source();
}

void __ucHAL_Bluetooth_function_transmit(uint8_t *buffer, uint32_t nbytes)
{
    __ucDrivers_LPUART_function_Transmit(buffer, nbytes);
}

//-------------------DISPLAY---------------------------------------------------------------
void __ucHAL_Display_configure()
{
    __ucDrivers_I2C_conf_IndependentClock_SysClkSource();
    __ucDrivers_I2C_enable_Clock();
    __ucDrivers_I2C_conf_Periphereal();

    __ucDrivers_I2C_enable_GPIO_Clock();
    __ucDrivers_I2C_conf_GPIO_Source();

    __ucDrivers_I2C_disable_Clock(); // Clock gating
}

void __ucHAL_Display_function_transmit(
    uint8_t i2c_addr, uint8_t *buffer, uint32_t nbytes)
{
    __ucDrivers_I2C_enable_Clock(); // Clock gating
    __ucDrivers_I2C_function_Transmit(i2c_addr, buffer, nbytes);
    __ucDrivers_I2C_disable_Clock(); // Clock gating
}

//-------------------BATTERY-----------------------------------------------------------------
void __ucHAL_Battery_configure()
{
    // Battery voltage level detection
    __ucDrivers_ADC_conf_IndependentClock_SysClkSource();
    __ucDrivers_ADC_enable_Clock();
    __ucDrivers_ADC_conf_Periphereal();

    __ucDrivers_ADC_enable_GPIO_Clock();
    __ucDrivers_ADC_conf_GPIO_Source();

    __ucDrivers_ADC_disable_Clock(); // Clock Gating

    // Connection to charger detection
}

//-------------------SLEEPTIMER------------------------------------------------------------
void __ucHAL_Sleeptimer_configure()
{
    __ucDrivers_TIM3_enable_Clock(); // Clock gating
    __ucDrivers_TIM3_conf_Periphereal();
    __ucDrivers_TIM3_disable_Clock(); // Clock gating
}

void __ucHAL_Sleeptimer_startOneShot(uint16_t initialValue, uint16_t autoReloadValue)
{
    __ucDrivers_TIM3_enable_Clock(); // Clock gating
    __ucDrivers_TIM3_function_startOneShotTimer(initialValue, autoReloadValue);
}

SleepTimerValues __ucHAL_Sleeptimer_stopOneShot()
{
    __ucDrivers_TIM3_function_stopOneShotTimer();
    SleepTimerValues timerValues;
    timerValues.counter = __ucDrivers_TIM3_function_readCounter();
    timerValues.updateInterruptOcurred =
        __ucDrivers_TIM3_function__readInterruptPendingStatus();

    if (timerValues.updateInterruptOcurred)
    {
        __ucDrivers_TIM3_function__clearInterruptPendingStatus();
        NVIC_ClearPendingIRQ(TIM3_IRQn);
    }

    __ucDrivers_TIM3_disable_Clock(); // Clock gating

    return timerValues;
}