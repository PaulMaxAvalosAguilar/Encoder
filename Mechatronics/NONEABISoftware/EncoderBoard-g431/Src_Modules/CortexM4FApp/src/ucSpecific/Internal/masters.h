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
void __ucDrivers_DMA_conf_Channel3ForI2CTX(void);

//-----------Functions
void __ucDrivers_DMA_function_StartChannel3ForI2CTX(uint8_t *buffer, uint32_t length); // This is internal shouldn't be public

#endif