#ifndef PERIPHEREALS_H
#define PERIPHEREALS_H

#include <inttypes.h>

///////////////M4 CORE MASTERS//////////////////////////////

///////////M4F CORE PERIPHEREALS////////////////////////////
//-------------------SYSTICK--------------------------------
void __ucDrivers_SYSTICK_function_enableTimer(void);
void __ucDrivers_SYSTICK_function_disableTimer(void);

//-------------------NVIC-----------------------------------
/*
            Implemented in core_cm4.h
*/
/////////STM32 SPECIFIC PERIPHEREALS////////////////////////

//-------------------FLASH----------------------------------
//-----------Clocks

//-----------Configurations

//-----------Functions
void __ucDrivers_FLASH_function_EnableCache(void);
void __ucDrivers_FLASH_function_Set1WSLatency(void);

//-------------------RCC------------------------------------
//-----------Clocks

//-----------Configurations

//-----------Functions
void __ucDrivers_RCC_function_TurnHSEOn(void);
void __ucDrivers_RCC_function_SetHSEAsPLLSource(void);
void __ucDrivers_RCC_function_SetPLLmn50MhzOn8MhzHSE(void);
void __ucDrivers_RCC_function_TurnPLLOn(void);
void __ucDrivers_RCC_function_EnablePLLROutput(void);
void __ucDrivers_RCC_function_SelectPLLAsClockSource(void);

void __ucDrivers_RCC_function_AHBPrescale97656KhzFrom50mhz(void);
void __ucDrivers_RCC_function_AHBPrescalerReset(void);

//-------------------PWR------------------------------------
void __ucDrivers_PWR_enable_Clock(void);
void __ucDrivers_PWR_disable_Clock(void);

void __ucDrivers_PWR_function_TurnOnLPR(void);
void __ucDrivers_PWR_function_TurnOffLPR(void); // Should be called after function_TurnOnLPR()

// I2C-------------------------------------------------------
#define I2C_DMA_Transmission_Enabled
//-----------Clocks
void __ucDrivers_I2C_enable_Clock(void);
void __ucDrivers_I2C_disable_Clock(void);
void __ucDrivers_I2C_enable_GPIO_Clock(void);
void __ucDrivers_I2C_disable_GPIO_Clock(void);
void __ucDrivers_I2C_conf_IndependentClock_SysClkSource(void);

//-----------Configurations
void __ucDrivers_I2C_conf_Periphereal(void);
void __ucDrivers_I2C_conf_GPIO_Source(void);

//-----------Functions
void __ucDrivers_I2C_function_Transmit(uint8_t i2c_addr, uint8_t *buffer, uint32_t nbytes);

//-------------------ADC------------------------------------
//-----------Clocks
void __ucDrivers_ADC_enable_Clock(void);
void __ucDrivers_ADC_disable_Clock(void);
void __ucDrivers_ADC_enable_GPIO_Clock(void);
void __ucDrivers_ADC_disable_GPIO_Clock(void);
void __ucDrivers_ADC_conf_IndependentClock_SysClkSource(void);

//-----------Configurations
void __ucDrivers_ADC_conf_Periphereal(void);
void __ucDrivers_ADC_conf_GPIO_Source(void);

//-----------Functions

//-------------------TIM3-----------------------------------
//-----------Clocks
void __ucDrivers_TIM3_enable_Clock(void);
void __ucDrivers_TIM3_disable_Clock(void);

//-----------Configurations
void __ucDrivers_TIM3_conf_Periphereal(void);

//-----------Functions
void __ucDrivers_TIM3_function_startOneShotTimer(uint16_t initialValue, uint16_t autoReloadValue);
uint16_t __ucDrivers_TIM3_function_readCounter(void);
uint8_t __ucDrivers_TIM3_function__readInterruptPendingStatus(void);
void __ucDrivers_TIM3_function__clearInterruptPendingStatus(void);
void __ucDrivers_TIM3_function_stopOneShotTimer(void);

//-------------------LPTIM----------------------------------
//-----------Clocks
void __ucDrivers_LPTIM_enable_Clock(void);
void __ucDrivers_LPTIM_disable_Clock(void);
void __ucDrivers_LPTIM_enable_GPIO_Clock(void);
void __ucDrivers_LPTIM_disable_GPIO_Clock(void);
void __ucDrivers_LPTIM_conf_IndependentClock_PCLKSource(void);

//-----------Configurations
void __ucDrivers_LPTIM_conf_Periphereal(void);
void __ucDrivers_LPTIM_conf_GPIO_Source(void);

#endif