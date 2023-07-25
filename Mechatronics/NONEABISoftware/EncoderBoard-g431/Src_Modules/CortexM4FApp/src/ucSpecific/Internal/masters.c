#include "masters.h"
#include "stm32g431xx.h"

//-------------------DMA-------------------------------------------------------------------------------------

void __ucDrivers_DMA_enable_Clock()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // Enable DMA1 clock
}

void __ucDrivers_DMA_disable_Clock()
{
    RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN; // Disable DMA1 clock
}

void __ucDrivers_DMAMUX_enable_Clock()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN; // Enable DMA multiplexer clock
}

void __ucDrivers_DMAMUX_disable_Clock()
{
    RCC->AHB1ENR &= ~RCC_AHB1ENR_DMAMUX1EN; // Disable DMA multiplexer clock
}

void __ucDrivers_DMA_conf_Channel3ForI2CTX()
{
    // Read from memory to periphereal
    DMA1_Channel3->CCR |= DMA_CCR_DIR;
    // Low Priority level
    DMA1_Channel3->CCR = (DMA1_Channel3->CCR & (~DMA_CCR_PL)) | (0b00 << DMA_CCR_PL_Pos);
    // circular mode disabled
    DMA1_Channel3->CCR &= ~(DMA_CCR_CIRC);
    // Number of bytes
    // set in __ucSpecific_function_DMA_Start_Channel3_For_I2C_TX

    //--------Memory Configuration-------------------------------------
    // Memory increment mode enabled
    DMA1_Channel3->CCR |= (DMA_CCR_MINC);
    // Memory size 8 bits
    DMA1_Channel3->CCR = (DMA1_Channel3->CCR &= (~DMA_CCR_MSIZE)) | (0b00 << DMA_CCR_MSIZE_Pos);
    // DMA source address
    // set in __ucSpecific_function_DMA_Start_Channel3_For_I2C_TX

    //--------Periphereal Configuration-------------------------------
    // Periphereal increment mode disabled
    DMA1_Channel3->CCR &= ~(DMA_CCR_PINC);
    // Perihphereal size 8 bits
    DMA1_Channel3->CCR = (DMA1_Channel3->CCR &= (~DMA_CCR_PSIZE)) | (0b00 << DMA_CCR_PSIZE_Pos);
    // Periphereal address
    DMA1_Channel3->CPAR = (uint32_t)&I2C2->TXDR;

    //--------Multiplexer---------------------------------------------
    // Request 19 I2C2_TX
    DMAMUX1_Channel2->CCR = (DMAMUX1_Channel2->CCR & (~DMAMUX_CxCR_DMAREQ_ID)) | (19 << DMAMUX_CxCR_DMAREQ_ID_Pos);
}

void __ucDrivers_DMA_conf_Channel3ForLPUART_RX()
{
    // Read from periphereal to memory
    DMA1_Channel1->CCR &= ~DMA_CCR_DIR;
    // High Priority level
    DMA1_Channel1->CCR = (DMA1_Channel1->CCR & (~DMA_CCR_PL)) | (0b10 << DMA_CCR_PL_Pos); // High Priority level
    // circular mode enabled
    DMA1_Channel1->CCR |= (DMA_CCR_CIRC);
    // Number of bytes
    // set in __ucSpecific_function_DMA_Start_Channel3_For_I2C_TX

    //--------Memory Configuration-------------------------------------
    // Memory increment mode enabled
    DMA1_Channel1->CCR |= (DMA_CCR_MINC);
    // Memory size 8 bits
    DMA1_Channel1->CCR = (DMA1_Channel3->CCR &= (~DMA_CCR_MSIZE)) | (0b00 << DMA_CCR_MSIZE_Pos);
    // DMA source address
    // set in __ucSpecific_function_DMA_Start_Channel3_For_I2C_TX

    //--------Periphereal Configuration-------------------------------
    // Periphereal increment mode disabled
    DMA1_Channel1->CCR &= ~(DMA_CCR_PINC);
    // Perihphereal size 8 bits
    DMA1_Channel1->CCR = (DMA1_Channel3->CCR &= (~DMA_CCR_PSIZE)) | (0b00 << DMA_CCR_PSIZE_Pos);
    // Periphereal address
    DMA1_Channel1->CPAR = (uint32_t)&LPUART1->RDR;

    //--------Multiplexer---------------------------------------------
    // Request 19 I2C2_TX
    DMAMUX1_Channel2->CCR = (DMAMUX1_Channel2->CCR & (~DMAMUX_CxCR_DMAREQ_ID)) | (19 << DMAMUX_CxCR_DMAREQ_ID_Pos);
}

void __ucDrivers_DMA_function_StartChannel3ForI2CTX(uint8_t *buffer, uint32_t length)
{
    DMA1_Channel3->CCR &= ~DMA_CCR_EN; // Channel disable
    DMA1_Channel3->CMAR = (uint32_t)buffer;
    DMA1_Channel3->CNDTR = length;    // DMA length
    DMA1_Channel3->CCR |= DMA_CCR_EN; // Channel enable
}