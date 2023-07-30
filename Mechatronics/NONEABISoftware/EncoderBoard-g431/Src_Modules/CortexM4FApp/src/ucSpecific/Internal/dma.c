#include "dma.h"
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

void __ucDrivers_DMA_conf_Channel1ForLPUART_RX(uint8_t *receiveBuffer, uint32_t length)
{
    // Read from periphereal to memory
    DMA1_Channel1->CCR &= ~DMA_CCR_DIR;
    // High Priority level
    DMA1_Channel1->CCR = (DMA1_Channel1->CCR & (~DMA_CCR_PL)) | (0b10 << DMA_CCR_PL_Pos); // High Priority level
    // circular mode enabled
    DMA1_Channel1->CCR |= (DMA_CCR_CIRC);
    // Number of bytes
    DMA1_Channel1->CNDTR = length;

    //--------Memory Configuration-------------------------------------
    // Memory increment mode enabled
    DMA1_Channel1->CCR |= (DMA_CCR_MINC);
    // Memory size 8 bits
    DMA1_Channel1->CCR = (DMA1_Channel1->CCR &= (~DMA_CCR_MSIZE)) | (0b00 << DMA_CCR_MSIZE_Pos);
    // DMA source address
    DMA1_Channel1->CMAR = (uint32_t)receiveBuffer;

    //--------Periphereal Configuration-------------------------------
    // Periphereal increment mode disabled
    DMA1_Channel1->CCR &= ~(DMA_CCR_PINC);
    // Perihphereal size 8 bits
    DMA1_Channel1->CCR = (DMA1_Channel1->CCR &= (~DMA_CCR_PSIZE)) | (0b00 << DMA_CCR_PSIZE_Pos);
    // Periphereal address
    DMA1_Channel1->CPAR = (uint32_t)&LPUART1->RDR;

    //--------Multiplexer---------------------------------------------
    // Request 34 LPUART_RX
    DMAMUX1_Channel0->CCR = (DMAMUX1_Channel1->CCR & (~DMAMUX_CxCR_DMAREQ_ID)) | (34 << DMAMUX_CxCR_DMAREQ_ID_Pos);

    DMA1_Channel1->CCR |= DMA_CCR_EN; // Channel enable
}

void __ucDrivers_DMA_conf_Channel2ForLPUART_TX()
{
    // Read from memory to periphereal
    DMA1_Channel2->CCR |= DMA_CCR_DIR;
    // Medium Priority level
    DMA1_Channel2->CCR = (DMA1_Channel2->CCR & (~DMA_CCR_PL)) | (0b01 << DMA_CCR_PL_Pos);
    // circular mode disabled
    DMA1_Channel2->CCR &= ~(DMA_CCR_CIRC);
    // Number of bytes
    // set in __ucSpecific_function_DMA_Start_Channel3_For_LPUART_TX

    //--------Memory Configuration-------------------------------------
    // Memory increment mode enabled
    DMA1_Channel2->CCR |= (DMA_CCR_MINC);
    // Memory size 8 bits
    DMA1_Channel2->CCR = (DMA1_Channel2->CCR &= (~DMA_CCR_MSIZE)) | (0b00 << DMA_CCR_MSIZE_Pos);
    // DMA source address
    // set in __ucSpecific_function_DMA_Start_Channel3_For_LPUART_TX

    //--------Periphereal Configuration-------------------------------
    // Periphereal increment mode disabled
    DMA1_Channel2->CCR &= ~(DMA_CCR_PINC);
    // Perihphereal size 8 bits
    DMA1_Channel2->CCR = (DMA1_Channel2->CCR &= (~DMA_CCR_PSIZE)) | (0b00 << DMA_CCR_PSIZE_Pos);
    // Periphereal address
    DMA1_Channel2->CPAR = (uint32_t)&LPUART1->TDR;

    //--------Multiplexer---------------------------------------------
    // Request 35 LPUART1_TX
    DMAMUX1_Channel1->CCR = (DMAMUX1_Channel2->CCR & (~DMAMUX_CxCR_DMAREQ_ID)) | (35 << DMAMUX_CxCR_DMAREQ_ID_Pos);
}

void __ucDrivers_DMA_conf_Channel3ForI2C_TX()
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
    DMAMUX1_Channel2->CCR = (DMAMUX1_Channel3->CCR & (~DMAMUX_CxCR_DMAREQ_ID)) | (19 << DMAMUX_CxCR_DMAREQ_ID_Pos);
}

void __ucDrivers_DMA_function_StartChannel2ForLPUART_TX(uint8_t *sendBuffer, uint32_t length)
{
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CMAR = (uint32_t)sendBuffer;
    DMA1_Channel2->CNDTR = length;
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}

void __ucDrivers_DMA_function_WaitForChannel2TransfereComplete()
{
    while (!(DMA1->ISR & DMA_ISR_TCIF2))
        ;
}

void __ucDrivers_DMA_function_ClearChannel2TransfereComplete()
{
    DMA1->IFCR |= DMA_IFCR_CTCIF2;
}

void __ucDrivers_DMA_function_StartChannel3ForI2CTX(uint8_t *sendBuffer, uint32_t length)
{
    DMA1_Channel3->CCR &= ~DMA_CCR_EN; // Channel disable
    DMA1_Channel3->CMAR = (uint32_t)sendBuffer;
    DMA1_Channel3->CNDTR = length;    // DMA length
    DMA1_Channel3->CCR |= DMA_CCR_EN; // Channel enable
}

void __ucDrivers_DMA_function_WaitForChannel3TransfereComplete()
{
    while (!(DMA1->ISR & DMA_ISR_TCIF3))
        ;
}

void __ucDrivers_DMA_function_ClearChannel3TransfereComplete()
{
    DMA1->IFCR |= DMA_IFCR_CTCIF3;
}