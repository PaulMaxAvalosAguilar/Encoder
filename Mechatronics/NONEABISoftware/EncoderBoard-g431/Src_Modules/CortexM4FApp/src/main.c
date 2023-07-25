#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "ucSpecific/Internal/masters.h"
#include "ucSpecific/Internal/drivers.h"
#include "ucSpecific/Public/hal.h"
#include "main.h"

#include "stm32g431xx.h"

#include "Processes/Priority2/displayTask.h"

int main(void)
{
  // OPTION BYTES
  // PG10 reset Input/Output
  // nBOOT0 = 1
  // nSWBOOT0  = 1 (BOOT0 taken from PB8/BOOT0 pin)
  // SRAM1 and CCM SRAM parity check disable
  // nBOOT1 = 1
  // Software window watchdog
  // Independent watchdog counter is running in stop mode
  // Software independent watchdog
  // No reset generated when entering the Stop mode
  // BOR_LEV threshold 1.7
  // Read protection not active

  //---------------------INCREASE CPU SPEED-------------------------

  // The HSI16 is used as system clock source after startup from Reset
  // Flash erase and programming is only possible in voltage scale range 1

  // PWR_CR1 Low power run not set, Voltage scaling range 1, low power mode= Stop 0 mode
  // PWR_CR5 Main regulator in range 1 normal mode
  // FLASH_ACR Instruction cache and data cache enabled, zero wait state*
  // RCC_CR All clocks off and not ready(hsi*), HSE not bypassed
  // RCC_CFGR HSI16 selected as system clock, sysclk & PCLK1 & PCLK2 not divided, MCO output disabled
  // RCC_PLLCFGR PLLn mult by 8, PLLm div by 1, no PLL sourcce, PLLR div by 2, PLLR disabled

  __ucHAL_SysFreq_configure_initialState();
  __ucHAL_DMA_configure();
  __ucHAL_Interrupts_configure();

  //-----------------Enable independent clocks-----------------

  RCC->CCIPR = (RCC->CCIPR & (~RCC_CCIPR_USART1SEL)) | (0b01 << RCC_CCIPR_USART1SEL_Pos); // System clock as USART1 clock

  //---------------------CONFIGURE GPIO---------------------

  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

  /*
  //TEST PINS CONFIGURATION
  //PA8
  GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE8)) | (0b01 << GPIO_MODER_MODE8_Pos); //General Purpose Output mode
  GPIOA->OSPEEDR = (GPIOA->OSPEEDR & (~GPIO_OSPEEDR_OSPEED8)) | (0b00 << GPIO_OSPEEDR_OSPEED8_Pos); //Low speed
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT8; //Push pull
  GPIOA->PUPDR = (GPIOA->PUPDR & (~GPIO_PUPDR_PUPD8)) | (0b00 << GPIO_PUPDR_PUPD8_Pos); //No Pull up down
  GPIOA->BSRR |= GPIO_BSRR_BS8;//Turn on PA8

  //PA9
  GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE9)) | (0b01 << GPIO_MODER_MODE9_Pos); //General Purpose Output mode
  GPIOA->OSPEEDR = (GPIOA->OSPEEDR & (~GPIO_OSPEEDR_OSPEED9)) | (0b00 << GPIO_OSPEEDR_OSPEED9_Pos); //Low speed
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT9; //Push pull
  GPIOA->PUPDR = (GPIOA->PUPDR & (~GPIO_PUPDR_PUPD9)) | (0b00 << GPIO_PUPDR_PUPD9_Pos); //No Pull up down
  GPIOA->BSRR |= GPIO_BSRR_BS9;//Turn on PA9
  */

  // UART1 GPIO CONFIGURATION
  // PA10 UART1_RX
  GPIOA->MODER = (GPIOA->MODER & (~GPIO_MODER_MODE10)) | (0b10 << GPIO_MODER_MODE10_Pos);             // Alternate function mode
  GPIOA->OTYPER &= (~GPIO_OTYPER_OT10);                                                               // Push pull
  GPIOA->OSPEEDR = (GPIOA->OSPEEDR & (~GPIO_OSPEEDR_OSPEED10)) | (0b00 << GPIO_OSPEEDR_OSPEED10_Pos); // Low speed
  GPIOA->PUPDR = (GPIOA->PUPDR & (~GPIO_PUPDR_PUPD10)) | (0b00 << GPIO_PUPDR_PUPD10_Pos);             // No pull up, no pull down
  GPIOA->AFR[1] = (GPIOA->AFR[1] & (~GPIO_AFRH_AFSEL10)) | (7 << GPIO_AFRH_AFSEL10_Pos);              // Alternate function 7

  // PB6 UART1_TX
  GPIOB->MODER = (GPIOB->MODER & (~GPIO_MODER_MODE6)) | (0b10 << GPIO_MODER_MODE6_Pos);             // Alternate function mode
  GPIOB->OTYPER &= (~GPIO_OTYPER_OT6);                                                              // Push pull
  GPIOB->OSPEEDR = (GPIOB->OSPEEDR & (~GPIO_OSPEEDR_OSPEED6)) | (0b00 << GPIO_OSPEEDR_OSPEED6_Pos); // Low speed
  GPIOB->PUPDR = (GPIOB->PUPDR & (~GPIO_PUPDR_PUPD6)) | (0b00 << GPIO_PUPDR_PUPD6_Pos);             // No pull up, no pull down
  GPIOB->AFR[0] = (GPIOB->AFR[0] & (~GPIO_AFRL_AFSEL6)) | (7 << GPIO_AFRL_AFSEL6_Pos);              // Alternate function 7

  // TIM2 GPIO CONFIGURATION

  // ENC ENABLE CONFIGURATION
  // PB9
  GPIOB->MODER = (GPIOB->MODER & (~GPIO_MODER_MODE9)) | (0b01 << GPIO_MODER_MODE9_Pos);             // General purpose outputMode
  GPIOB->OTYPER &= (~GPIO_OTYPER_OT9);                                                              // Push pull
  GPIOB->OSPEEDR = (GPIOB->OSPEEDR & (~GPIO_OSPEEDR_OSPEED9)) | (0b00 << GPIO_OSPEEDR_OSPEED9_Pos); // Low speed
  GPIOB->PUPDR = (GPIOB->PUPDR & (~GPIO_PUPDR_PUPD9)) | (0b00 << GPIO_PUPDR_PUPD9_Pos);             // No pull up, no pull down
  GPIOB->BSRR |= GPIO_BSRR_BR9;                                                                     // Turn on encoder

  // ENC CONNECTED CONFIGURATION
  // PB12
  GPIOB->MODER = (GPIOB->MODER & (~GPIO_MODER_MODE12)) | (0b00 << GPIO_MODER_MODE12_Pos); // Input mode
  GPIOB->PUPDR = (GPIOB->PUPDR & (~GPIO_PUPDR_PUPD12)) | (0b10 << GPIO_PUPDR_PUPD12_Pos); // Pull down

  //---------------------CONFIGURE UART-------------------------
  /*
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN; //Enable UART1 clock

  //UART1 CONFIGURATION
  USART1->CR1 &= ~USART_CR1_FIFOEN;//FIFO mode disabled
  USART1->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1);//1 start bit, 8 data bits
  USART1->CR1 &= ~USART_CR1_OVER8;//Oversampling by 16
  USART1->CR1 &= ~USART_CR1_PCE;//No parity
  USART1->CR1 |= USART_CR1_IDLEIE;//IDLE interrupt enable
  USART1->CR1 |= USART_CR1_TE;//Enable transmiter
  USART1->CR1 |= USART_CR1_RE;//Enable receiver
  USART1->CR2 = (USART1->CR2 & ~(USART_CR2_STOP)) | (0b00 << USART_CR2_STOP_Pos);//1 Stop bit
  USART1->CR3 &= ~USART_CR3_ONEBIT;//Three sample bit method
  USART1->CR3 &= ~USART_CR3_CTSE;//CTS disabled
  USART1->CR3 &= ~USART_CR3_RTSE;//RTS disabled
  USART1->CR3 |= USART_CR3_OVRDIS;//Overrun Disable
  USART1->CR3 |= USART_CR3_DMAT;//Enable DMA transmit
  USART1->CR3 |= USART_CR3_DMAR;//Enable DMA receive
  USART1->PRESC = (USART1->PRESC & (~USART_PRESC_PRESCALER)) | (0b0000 << USART_PRESC_PRESCALER_Pos);//Input clock not divided
  USART1->BRR = 434;//50,000,000/ 434 = 115,200
  USART1->CR1 |= USART_CR1_UE;//Enable USART
  */

  //-------------------CONFIGURE DMA---------------------------

  /*
  //DMA UART1 RX CONFIGURATION
  DMA1_Channel1->CCR &= ~DMA_CCR_DIR; //read from periphereal
  DMA1_Channel1->CCR = (DMA1_Channel1->CCR & (~DMA_CCR_PL)) | (0b10 << DMA_CCR_PL_Pos); //High Priority level
  DMA1_Channel1->CCR |= DMA_CCR_CIRC;//circular mode enabled
  DMA1_Channel1->CNDTR = UART_RX_BUFFER_LEN;//DMA length

  DMA1_Channel1->CPAR = (uint32_t)&USART1->RDR;//DMA source address
  DMA1_Channel1->CCR &= ~(DMA_CCR_PINC);//Periphereal increment mode disabled
  DMA1_Channel1->CCR = (DMA1_Channel1->CCR &= (~DMA_CCR_PSIZE)) | (0b00 << DMA_CCR_PSIZE_Pos); //Perihphereal size 8 bits

  DMA1_Channel1->CMAR = (uint32_t)receiveBuffer;//DMA destination address
  DMA1_Channel1->CCR |= (DMA_CCR_MINC);//Memory increment mode enabled
  DMA1_Channel1->CCR = (DMA1_Channel1->CCR &= (~DMA_CCR_MSIZE)) | (0b00 << DMA_CCR_MSIZE_Pos); //Memory size 8 bits

  DMAMUX1_Channel0->CCR = (DMAMUX1_Channel0->CCR & (~DMAMUX_CxCR_DMAREQ_ID)) | (24 << DMAMUX_CxCR_DMAREQ_ID_Pos);//Requppest 24

  DMA1_Channel1->CCR |= DMA_CCR_EN;//Channel enable

  //DMA UART1 TX CONFIGURATION
  DMA1_Channel2->CCR |= DMA_CCR_DIR; //read from memory
  DMA1_Channel2->CCR = (DMA1_Channel2->CCR & (~DMA_CCR_PL)) | (0b01 << DMA_CCR_PL_Pos); //Medium Priority level
  DMA1_Channel2->CCR &= ~(DMA_CCR_CIRC);//circular mode disabled

  DMA1_Channel2->CPAR = (uint32_t)&USART1->TDR;//DMA destination address
  DMA1_Channel2->CCR &= ~(DMA_CCR_PINC);//Periphereal increment mode disabled
  DMA1_Channel2->CCR = (DMA1_Channel2->CCR &= (~DMA_CCR_PSIZE)) | (0b00 << DMA_CCR_PSIZE_Pos); //Perihphereal size 8 bits

  DMA1_Channel2->CCR |= (DMA_CCR_MINC);//Memory increment mode enabled
  DMA1_Channel2->CCR = (DMA1_Channel2->CCR &= (~DMA_CCR_MSIZE)) | (0b00 << DMA_CCR_MSIZE_Pos); //Memory size 8 bits

  DMAMUX1_Channel1->CCR = (DMAMUX1_Channel1->CCR & (~DMAMUX_CxCR_DMAREQ_ID)) | (25 << DMAMUX_CxCR_DMAREQ_ID_Pos);//Request 25
  */

  //---------------------CONFIGURE TIM3-------------------------
  /*
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;//Enable TIM3 clock

  //TIM3 CONFIGURATION
  TIM3->CR1 |= TIM_CR1_OPM;//One pulse mode
  TIM3->DIER |= TIM_DIER_UIE;//Enable update interrupt
  TIM3->CR1 |= TIM_CR1_URS;//Only counter overflow generates interrupt
  TIM3->PSC = 9999; //Preescaler for .051" at 195.3125Khz / actual value = TIM3->PSC +1
  TIM3->EGR = TIM_EGR_UG;//Generate update
  TIM3->CNT = 0;

  RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM3EN;//Disable TIM3 clock
  */

  //---------------------CONFIGURE EXTI-------------------------
  /*
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;//Enable SYSCFG clock

  SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] & (~SYSCFG_EXTICR4_EXTI12)) | (0b0001 << SYSCFG_EXTICR4_EXTI12_Pos);

  EXTI->IMR1 |= 1 << EXTI_IMR1_IM12_Pos;//Enable Exti 12
  EXTI->RTSR1 |= 1 << EXTI_RTSR1_RT12_Pos;//Enable Exti 12 rising trigger
  EXTI->FTSR1 |= 1 << EXTI_FTSR1_FT12_Pos;//Enable Exti 12 falling trigger
  */

  //---------------------CONFIGURE PWR---------------------------

  //---------------------CONFIGURE ADC---------------------------

  //---------------------CONFIGURE CORDIC------------------------

  /*
  RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;

  //CORDIC CONFIGURATION
  CORDIC->CSR = (CORDIC->CSR & (~CORDIC_CSR_FUNC)) | (0 << CORDIC_CSR_FUNC_Pos);//Cosine function selected
  CORDIC->CSR &= ~CORDIC_CSR_ARGSIZE;//Arg width 32 bit
  CORDIC->CSR &= ~CORDIC_CSR_RESSIZE;//Arg width 32 bit
  CORDIC->CSR &= ~CORDIC_CSR_NARGS;//Only one 32 bit write
  CORDIC->CSR |= CORDIC_CSR_NRES;//Two results, two reads necessary
  */

  //---------------------CONFIGURE RTOS-------------------------

  /*
  uartTXQueue = xQueueCreate(TX_QUEUE_SIZE, sizeof(uartTXData_t));
  lcdQueue = xQueueCreate(LCD_QUEUE_SIZE, sizeof(lcdData_t));

  xTaskCreate(uartRXTask, "uartRXTask",100, NULL, 3, &uartRXTaskHandle);
  xTaskCreate(uartTXTask, "uartTXTask",100, NULL, 2, NULL);
  xTaskCreate(lcdTask,"lcdTask",100, NULL, 2, NULL);
  xTaskCreate(adcFreeTask, "adcFreeTask", 100, NULL,1, &adcFreeTaskHandle);
  */

  //---------------------CONFIGURE NVIC---------------------------

  /*

    */

  //---------------------START RTOS-------------------------------
  __displayTask_init();
  vTaskStartScheduler();

  while (1)
  {
  }
}
