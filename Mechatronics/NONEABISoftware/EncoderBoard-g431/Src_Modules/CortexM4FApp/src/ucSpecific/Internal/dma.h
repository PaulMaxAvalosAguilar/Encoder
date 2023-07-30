#ifndef MASTERS_H
#define MASTERS_H

#include <inttypes.h>

//-------------------DMA------------------------------------
//-----------Clocks
void __ucDrivers_DMA_enable_Clock(void);
void __ucDrivers_DMA_disable_Clock(void);
void __ucDrivers_DMAMUX_enable_Clock(void);
void __ucDrivers_DMAMUX_disable_Clock(void);

//-----------Configurations
void __ucDrivers_DMA_conf_Channel1ForLPUART_RX(uint8_t *receiveBuffer, uint32_t length);
void __ucDrivers_DMA_conf_Channel2ForLPUART_TX(void);
void __ucDrivers_DMA_conf_Channel3ForI2C_TX(void);

//-----------Functions
// This are internal shouldn't be public
void __ucDrivers_DMA_function_StartChannel2ForLPUART_TX(uint8_t *sendBuffer, uint32_t length);
void __ucDrivers_DMA_function_WaitForChannel2TransfereComplete(void);
void __ucDrivers_DMA_function_ClearChannel2TransfereComplete(void);
void __ucDrivers_DMA_function_StartChannel3ForI2CTX(uint8_t *sendBuffer, uint32_t length);
void __ucDrivers_DMA_function_WaitForChannel3TransfereComplete(void);
void __ucDrivers_DMA_function_ClearChannel3TransfereComplete(void);

#endif