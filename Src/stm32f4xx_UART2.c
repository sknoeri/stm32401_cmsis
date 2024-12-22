#include "stm32f4xx_UART2.h"
/**
  * @brief USART2 Initialization Function
  * @param uint32_t baudRate
  * @param uint8_t UseDMA 0 or 1
  * @retval None
  */
void USART2_UART_Init(uint32_t baudRate, uint8_t UseDMA)
{
  RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;// enables the clock for USART2
  // GPIOA Pins konfigurieren (PA2: TX, PA3: RX)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER |= (2U << (2 * 2)) | (2U << (3 * 2));  // PA2 und PA3 auf Alternate Function
  GPIOA->AFR[0] |= (7U << (4 * 2)) | (7U << (4 * 3)); // AF7 für USART2

  USART2->CR1 &= ~ USART_CR1_M; // swts the wordlength to 8 bit
  USART2->CR1 &= ~ USART_CR1_OVER8; // slect oversampling mode 16 
  USART2->CR1 &= ~ USART_CR1_PCE; // parity control disabled
  USART2->CR2 &= ~ USART_CR2_STOP; // makes 1 stop bit
  if(UseDMA>0){
    USART2->CR3 |= USART_CR3_DMAT; // enables the DMA trasmition
  }else{
    USART2->CR3 = 0UL;
  }
  // set the DMAT in USART_CR3 must figure out how
  // Set the baud rate
  // Compute the Mantis register and the fraction register from USARTDIV=fck/(8*(2-OVER()*baudrate))
  /*the formula CRR = (fck + (baud / 2)) / baud; is equivalent with:
    USARTDIV = ((fck+baudRate/2)*1000)/16/baudRate; 
    *1000 to keep 3 stellen nach dem komma für frac berechung
    +baud/2 um kinen rundungs fehler zu machen bei der konvertierung zu uint32_t
    manti = USARTDIV/1000; // extracts the mantissa ie the int part of USARTDIV
    frac = (USARTDIV % 1000)*16/1000; // extracts the fraction ie the fractional positions of USARTDIV
    USART2->BRR |= (manti<<4)|frac;
  */
  // fck = SystemCoreClock/2; fck is the APB1 clock
  uint32_t fck, USARTDIV;
  fck = SystemCoreClock/2;
  USARTDIV = (fck + (baudRate/ 2))/baudRate; // works only if OVER1 = 0;
  USART2->BRR |= USARTDIV;

  // Confiugres the USART mode 
  USART2->CR1 |= USART_CR1_TE|USART_CR1_RE; // enables recive and transmition
  USART2->CR1 |= USART_CR1_UE; // enables the usart2  

  /*
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }*/
}
/**
  * @brief USART2 Send char withoud DMAStream
  * @param char c
  * @retval None
  */
void UART2_SendChar(char c) {
  // Warten, bis das Übertragungsdatenregister leer ist (TXE-Flag gesetzt)
  while (!(USART2->SR & USART_SR_TXE)); 

  // Zeichen in das Datenregister schreiben
  USART2->DR = (uint8_t)c;

  // Optional: Warten, bis das Zeichen vollständig gesendet wurde (TC-Flag gesetzt)
  while (!(USART2->SR & USART_SR_TC)); 
}
/**
  * @brief USART2 initalize DMAStream6 for Transmission
  * @param None
  * @retval None
  */
void DMA_Init_USART2_TX(void){
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // enable DMA clock
  DMA1_Stream6->CR =0; //PSIZE is 00 so 8bit word lenth
  DMA1_Stream6->CR |= (4UL << DMA_SxCR_CHSEL_Pos); //CHannel 4 is used for sream6 and _USART2
  DMA1_Stream6->CR |= DMA_SxCR_DIR_0; // Form Merory to periferie
  DMA1_Stream6->CR |= DMA_SxCR_MINC; // increments the memory pointer
  DMA1_Stream6->CR |= DMA_SxCR_TCIE; // Transfer interupt enable
  DMA1_Stream6->CR |= DMA_SxCR_PL_1; // high priory of DMA stream
  // To enabl DMA transmission the Memory and Periferial pointer must be set
  // The size to transfer must be of the data must be set
  // The the DMA must be enable 
}
/**
  * @brief USART2 Send char array with DMAStream6
  * @param char *data
  * @param uint16_t size
  * @retval None
  */
void USART2_Send_DMA(char *data, uint16_t size){
  DMA1_Stream6->M0AR = (uint32_t)data;
  DMA1_Stream6->PAR = (uint32_t)&USART2->DR; // Adres of drive register
  DMA1_Stream6->NDTR = size;
  DMA1_Stream6->CR |= DMA_SxCR_EN; // enable DMA

  NVIC_SetPriority(DMA1_Stream6_IRQn,1);
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

void DMA1_Stream6_IRQHandler(void) {
  if (DMA1->HISR & DMA_HISR_TCIF6) { // Übertragungs-Interrupt prüfen
    DMA1->HIFCR |= DMA_HIFCR_CTCIF6; // Interrupt-Flag löschen
    // Übertragung abgeschlossen
  }
}
