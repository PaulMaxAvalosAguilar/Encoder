#include "drivers.h"
#include "masters.h"
#include "stm32g431xx.h"

//-------------------FLASH-------------------------------------------------------------------------------------
void __ucDrivers_FLASH_function_EnableCache()
{
    FLASH->ACR |= FLASH_ACR_PRFTEN;
    FLASH->ACR |= FLASH_ACR_ICEN;
    FLASH->ACR |= FLASH_ACR_DCEN;
}

void __ucDrivers_FLASH_function_Set1WSLatency()
{
    // Wait states for less than 90 MHz at VCore Range 1 normal mode
    FLASH->ACR = (FLASH->ACR & (~FLASH_ACR_LATENCY)) | FLASH_ACR_LATENCY_1WS;
}

void __ucDrivers_RCC_function_TurnHSEOn()
{
    RCC->CR |= RCC_CR_HSEON;             // Turn on
    while (!(RCC->CR & (RCC_CR_HSERDY))) // Wait till ready
        ;
}

void __ucDrivers_RCC_function_SetHSEAsPLLSource()
{
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
}

void __ucDrivers_RCC_function_SetPLLmn50MhzOn8MhzHSE()
{
    RCC->PLLCFGR =
        (RCC->PLLCFGR &
         ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN)) |
        (0b01 << RCC_PLLCFGR_PLLM_Pos) | (25 << RCC_PLLCFGR_PLLN_Pos); // M = 2, N = 25 50 Mhz SYSCLK
}

void __ucDrivers_RCC_function_TurnPLLOn()
{
    RCC->CR |= RCC_CR_PLLON;
    while (!RCC_CR_PLLRDY)
        ;
}

void __ucDrivers_RCC_function_EnablePLLROutput()
{
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
}

void __ucDrivers_RCC_function_SelectPLLAsClockSource()
{
    // Select PLL as system clocksource and wait till selected
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW_PLL)) | (RCC_CFGR_SW_PLL);
    while (((RCC->CFGR) & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;
}

void __ucDrivers_RCC_function_AHBPrescale97656KhzFrom50mhz()
{
    // Prescaler set to 512
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_HPRE)) | (0b1111 << RCC_CFGR_HPRE_Pos); // CPU freq 97.65625 Khz
}

void __ucDrivers_RCC_function_AHBPrescalerReset()
{
    // Prescaler set to 1
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_HPRE)) | (0b0000 << RCC_CFGR_HPRE_Pos);
}

//-------------------SYSTICK-----------------------------------------------------------------------------------
void __ucDrivers_SYSTICK_function_enableTimer()
{
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void __ucDrivers_SYSTICK_function_disableTimer()
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // Stop the SysTick
}

//-------------------PWR---------------------------------------------------------------------------------------
void __ucDrivers_PWR_enable_Clock(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
}

void __ucDrivers_PWR_disable_Clock(void)
{
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_PWREN;
}

void __ucDrivers_PWR_function_TurnOnLPR(void)
{
    __ucDrivers_PWR_enable_Clock();
    PWR->CR1 |= PWR_CR1_LPR;
    while (!(PWR->SR2 & PWR_SR2_REGLPS))
        ; // Wait till low power regulator started
    while (!(PWR->SR2 & PWR_SR2_REGLPF))
        ; // Wait till regulator is in low power mode
}

void __ucDrivers_PWR_function_TurnOffLPR(void)
{
    PWR->CR1 &= ~PWR_CR1_LPR;
    while ((PWR->SR2 & PWR_SR2_REGLPF))
        ;
    __ucDrivers_PWR_disable_Clock();
}

//-------------------I2C---------------------------------------------------------------------------------------

void __ucDrivers_function_I2C_Transmit_DMA(uint8_t i2c_addr, uint8_t *buffer, uint32_t nbytes);
void __ucDrivers_function_I2C_Transmit_POLLING(uint8_t i2c_addr, uint8_t *buffer, uint32_t nbytes);

void __ucDrivers_I2C_enable_Clock()
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
}

void __ucDrivers_I2C_disable_Clock()
{
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C2EN;
}

void __ucDrivers_I2C_enable_GPIO_Clock()
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
}

void __ucDrivers_I2C_disable_GPIO_Clock()
{
    RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN;
}

void __ucDrivers_I2C_conf_IndependentClock_SysClkSource()
{
    RCC->CCIPR = (RCC->CCIPR & (~RCC_CCIPR_I2C2SEL)) | (0b01 << RCC_CCIPR_I2C2SEL_Pos); // System clock as I2C
}

void __ucDrivers_I2C_conf_Periphereal()
{
    I2C2->CR1 &= ~I2C_CR1_ANFOFF;             // Aanalog noise filter enabled
    I2C2->CR1 &= ~I2C_CR1_NOSTRETCH;          // Clock stretching enabled
    I2C2->TIMINGR = 0x00401959 & 0xF0FFFFFFU; // Set timings 400kz
    I2C2->CR2 &= ~I2C_CR2_ADD10;              // The master operatines in 7 bit addressing mode
    I2C2->CR1 |= I2C_CR1_PE;                  // Enable periphereal
}

void __ucDrivers_I2C_conf_GPIO_Source()
{
    // PA8 I2C2_SDA
    GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE8)) | (0b10 << GPIO_MODER_MODE8_Pos);             // Alternate function mode
    GPIOA->OTYPER |= GPIO_OTYPER_OT8;                                                                 // Open drain
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & (~GPIO_OSPEEDR_OSPEED8)) | (0b00 << GPIO_OSPEEDR_OSPEED8_Pos); // Low speed
    GPIOA->PUPDR = (GPIOA->PUPDR & (~GPIO_PUPDR_PUPD8)) | (0b01 << GPIO_PUPDR_PUPD8_Pos);             // Pull up
    GPIOA->AFR[1] = (GPIOA->AFR[1] & (~GPIO_AFRH_AFSEL8)) | (4 << GPIO_AFRH_AFSEL8_Pos);              // Alternate function 4

    // PA9 I2C2_SCL
    GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE9)) | (0b10 << GPIO_MODER_MODE9_Pos);             // Alternate function mode
    GPIOA->OTYPER |= GPIO_OTYPER_OT9;                                                                 // Open drain
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & (~GPIO_OSPEEDR_OSPEED9)) | (0b00 << GPIO_OSPEEDR_OSPEED9_Pos); // Low speed
    GPIOA->PUPDR = (GPIOA->PUPDR & (~GPIO_PUPDR_PUPD9)) | (0b01 << GPIO_PUPDR_PUPD9_Pos);             // Pull up
    GPIOA->AFR[1] = (GPIOA->AFR[1] & (~GPIO_AFRH_AFSEL9)) | (4 << GPIO_AFRH_AFSEL9_Pos);              // Alternate function 4
}

void __ucDrivers_I2C_function_Transmit(uint8_t i2c_addr, uint8_t *buffer, uint32_t nbytes)
{
#ifdef I2C_DMA_Transmission_Enabled
    __ucDrivers_function_I2C_Transmit_DMA(i2c_addr, buffer, nbytes);
#else
    __ucDrivers_function_I2C_Transmit_POLLING(i2c_addr, buffer, nbytes);
#endif
}

void __ucDrivers_function_I2C_Transmit_DMA(uint8_t i2c_addr, uint8_t *buffer, uint32_t nbytes)
{
    while (I2C2->ISR & I2C_ISR_BUSY)
        ; // Wait till bus is free

    I2C2->CR2 = (I2C2->CR2 & (~I2C_CR2_SADD)) | (i2c_addr << I2C_CR2_SADD_Pos);   // Slave address
    I2C2->CR2 &= ~I2C_CR2_RD_WRN;                                                 // Master requests a write transfer
    I2C2->CR2 = (I2C2->CR2 & (~I2C_CR2_NBYTES)) | (nbytes << I2C_CR2_NBYTES_Pos); // Nuber of bytes to be transfered
    I2C2->CR2 |= I2C_CR2_AUTOEND;                                                 // Automatic end mode

    __ucDrivers_DMA_function_StartChannel3ForI2CTX(buffer, nbytes);

    I2C2->CR1 |= I2C_CR1_TXDMAEN; // Enable DMA transmit
    I2C2->CR2 |= I2C_CR2_START;   // Start generation
    while (!(DMA1->ISR & DMA_ISR_TCIF3))
        ;                            // Wait till transfere complete
    DMA1->IFCR |= DMA_IFCR_CTCIF3;   // Clear transfere complete
    I2C2->CR1 &= (~I2C_CR1_TXDMAEN); // Disable DMA transmit
}

void __ucDrivers_function_I2C_Transmit_POLLING(uint8_t i2c_addr, uint8_t *buffer, uint32_t nbytes)
{
    while (I2C2->ISR & I2C_ISR_BUSY)
        ; // Wait till bus is free

    I2C2->CR2 = (I2C2->CR2 & (~I2C_CR2_SADD)) | (i2c_addr << I2C_CR2_SADD_Pos);   // Slave address
    I2C2->CR2 &= ~I2C_CR2_RD_WRN;                                                 // Master requests a write transfer
    I2C2->CR2 = (I2C2->CR2 & (~I2C_CR2_NBYTES)) | (nbytes << I2C_CR2_NBYTES_Pos); // Nuber of bytes to be transfered
    I2C2->CR2 |= I2C_CR2_AUTOEND;                                                 // Automatic end mode
    I2C2->CR2 |= I2C_CR2_START;                                                   // Start generation

    while ((I2C2->CR2 & I2C_CR2_START))
        ; // Wait till I2C is cleared

    for (uint8_t i = 0; i < nbytes; i++)
    {
        while (!(I2C2->ISR & I2C_ISR_TXIS))
            ;
        I2C2->TXDR = *buffer++; // Send TXIS
    }
}

//-------------------ADC---------------------------------------------------------------------------------------

void __ucDrivers_ADC_enable_Clock(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
}

void __ucDrivers_ADC_disable_Clock(void)
{
    RCC->AHB2ENR &= ~RCC_AHB2ENR_ADC12EN;
}

void __ucDrivers_ADC_enable_GPIO_Clock(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
}

void __ucDrivers_ADC_disable_GPIO_Clock(void)
{
    RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOBEN;
}

void __ucDrivers_ADC_conf_IndependentClock_SysClkSource(void)
{
    RCC->CCIPR = (RCC->CCIPR & (~RCC_CCIPR_ADC12SEL)) | (0b10 << RCC_CCIPR_ADC12SEL_Pos); // Sysclock selected as ADC 1 & 2 clock
}

void __ucDrivers_ADC_conf_Periphereal(void)
{
    ADC12_COMMON->CCR = (ADC12_COMMON->CCR & (~ADC_CCR_CKMODE)) | (0b00 << ADC_CCR_CKMODE_Pos); // Adc generated at product level
    ADC12_COMMON->CCR = (ADC12_COMMON->CCR & (~ADC_CCR_PRESC)) | (0b0000 << ADC_CCR_PRESC_Pos); // Input ADC clock not divided

    // TODO: DISABLE THIS
    ADC12_COMMON->CCR |= ADC_CCR_VREFEN; // Vrefint enable

    // ADC1 CONFIGURATION
    ADC1->CR &= ~ADC_CR_DEEPPWD; // Exit deep power down mode
    ADC1->CR |= ADC_CR_ADVREGEN; // Enable ADC internal voltage regulator
    for (int i = 0; i < 500; i++)
        ; // TADCVREG_STUP 20us wait  //TODO: Calculate from requency

    //  ADC1->CR |= ADC_CR_DEEPPWD;//Writing DEEPPWD=1 automatically disables the ADC voltage regulator and bit ADVREGEN

    // ADC calibration. Must be done with ADC_CR_ADEN disabled
    ADC1->CR &= ~ADC_CR_ADCALDIF; // Calibration in single ended mode
    ADC1->CR |= ADC_CR_ADCAL;     // Initiate calibration
    while (ADC1->CR & ADC_CR_ADCAL)
        ; // Wait while calibratoin is in progress
    // TODO: Read calibration factor

    // Channel single or differential configuration. Must be done with ADC_CR_ADEN disabled
    ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_15; // IN5 Single ended

    ADC1->ISR |= ADC_ISR_ADRDY; // Clear ADRDY bit
    ADC1->CR |= ADC_CR_ADEN;    // Enable ADC1;
    while (!(ADC1->ISR & ADC_ISR_ADRDY))
        ; // Wait till ADC is ready4
          // TODO: Clear the ADRDY bitn in the ADC_ISR by writing 1
}

void __ucDrivers_ADC_conf_GPIO_Source(void)
{
    // PB0 ADC1_IN15
    GPIOB->MODER = (GPIOB->MODER & (~GPIO_MODER_MODE0)) | (0b11 << GPIO_MODER_MODE0_Pos); // Analog mode
    GPIOB->PUPDR = (GPIOB->PUPDR & (~GPIO_PUPDR_PUPD0)) | (0b00 << GPIO_PUPDR_PUPD0_Pos); // No pull up, no pull down
}

//-------------------TIM3--------------------------------------------------------------------------------------

void __ucDrivers_TIM3_enable_Clock(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
}

void __ucDrivers_TIM3_disable_Clock(void)
{
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM3EN;
}

void __ucDrivers_TIM3_conf_Periphereal(void)
{
    // TIM3 CONFIGURATION
    TIM3->CR1 |= TIM_CR1_OPM;   // One pulse mode
    TIM3->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM3->CR1 |= TIM_CR1_URS;   // Only counter overflow generates interrupt
    TIM3->PSC = 9999;           // Preescaler for .051" at 195.3125Khz / actual value = TIM3->PSC +1
    TIM3->EGR = TIM_EGR_UG;     // Generate update
    TIM3->CNT = 0;
}

void __ucDrivers_TIM3_function_startOneShotTimer(uint16_t initialValue, uint16_t autoReloadValue)
{
    TIM3->CNT = initialValue;
    TIM3->ARR = autoReloadValue;
    TIM3->CR1 |= TIM_CR1_CEN;
}

uint16_t __ucDrivers_TIM3_function_readCounter()
{
    return TIM3->CNT;
}

uint8_t __ucDrivers_TIM3_function__readInterruptPendingStatus()
{
    return TIM3->SR & TIM_SR_UIF;
}

void __ucDrivers_TIM3_function__clearInterruptPendingStatus()
{
    TIM3->SR = ~TIM_SR_UIF;
}

void __ucDrivers_TIM3_function_stopOneShotTimer()
{
    TIM3->CR1 &= ~TIM_CR1_CEN;
}

//-------------------LIPTM------------------------------------------------------------------------------------

void __ucDrivers_LPTIM_enable_Clock()
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
}

void __ucDrivers_LPTIM_disable_Clock()
{
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_LPTIM1EN;
}

void __ucDrivers_LPTIM_enable_GPIO_Clock()
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
}

void __ucDrivers_LPTIM_disable_GPIO_Clock()
{
    RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOBEN;
}

void __ucDrivers_LPTIM_conf_IndependentClock_PCLKSource()
{
    RCC->CCIPR = (RCC->CCIPR & (~RCC_CCIPR_LPTIM1SEL)) | (0b00 << RCC_CCIPR_LPTIM1SEL_Pos); // PCLK selected as LPTIM1 clock
}

void __ucDrivers_LPTIM_conf_Periphereal()
{
    LPTIM1->CFGR = (LPTIM1->CFGR & (~LPTIM_CFGR_CKFLT)) | (0b11 << LPTIM_CFGR_CKFLT_Pos); // 8 clock periods for valid transition
    LPTIM1->CFGR &= ~LPTIM_CFGR_CKSEL;                                                    ////LPTIM clocked by internal clock source
    LPTIM1->CFGR = (LPTIM1->CFGR & (~LPTIM_CFGR_CKPOL)) | (0b10 << LPTIM_CFGR_CKPOL_Pos); // both edges are active edges
    LPTIM1->CFGR |= LPTIM_CFGR_COUNTMODE;                                                 // External input 1
    LPTIM1->CFGR |= LPTIM_CFGR_ENC;                                                       // Encoder mode enable
    LPTIM1->CR |= LPTIM_CR_ENABLE;                                                        // LPTIM1 Enable
    LPTIM1->ICR |= LPTIM_ICR_ARROKCF;
    LPTIM1->ARR = 65535;
    while (!(LPTIM1->ISR & LPTIM_ISR_ARROK))
        ;
    LPTIM1->CR |= LPTIM_CR_CNTSTRT; // Timer start in Continuous mode
}

void __ucDrivers_LPTIM_conf_GPIO_Source()
{
    // PB5 LPTIM_IN1
    GPIOB->MODER = (GPIOB->MODER & (~GPIO_MODER_MODE5)) | (0b10 << GPIO_MODER_MODE5_Pos);             // Alternate function mode
    GPIOB->OTYPER &= (~GPIO_OTYPER_OT5);                                                              // Push pull
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & (~GPIO_OSPEEDR_OSPEED5)) | (0b00 << GPIO_OSPEEDR_OSPEED5_Pos); // Low speed
    GPIOB->PUPDR = (GPIOB->PUPDR & (~GPIO_PUPDR_PUPD5)) | (0b00 << GPIO_PUPDR_PUPD5_Pos);             // No pull up, no pull down
    GPIOB->AFR[0] = (GPIOB->AFR[0] & (~GPIO_AFRL_AFSEL5)) | (11 << GPIO_AFRL_AFSEL5_Pos);             // Alternate function 11

    // PB7 LPTIM_IN2
    GPIOB->MODER = (GPIOB->MODER & (~GPIO_MODER_MODE7)) | (0b10 << GPIO_MODER_MODE7_Pos);             // Alternate function mode
    GPIOB->OTYPER &= (~GPIO_OTYPER_OT7);                                                              // Push pull
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & (~GPIO_OSPEEDR_OSPEED7)) | (0b00 << GPIO_OSPEEDR_OSPEED7_Pos); // Low speed
    GPIOB->PUPDR = (GPIOB->PUPDR & (~GPIO_PUPDR_PUPD7)) | (0b00 << GPIO_PUPDR_PUPD7_Pos);             // No pull up, no pull down
    GPIOB->AFR[0] = (GPIOB->AFR[0] & (~GPIO_AFRL_AFSEL7)) | (11 << GPIO_AFRL_AFSEL7_Pos);             // Alternate function 11
}