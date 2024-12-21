/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
  
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

static void GPIO_Init(void);
static void USART2_UART_Init(uint32_t);
static void DMA_Init_USART2_TX(void);
void USART2_Send_DMA(char *, uint16_t);
void UART2_SendChar(char);

int main(void)
{

  GPIO_Init();
  USART2_UART_Init(115200);
  DMA_Init_USART2_TX();
  char data [5] = {'C','B','D','E','\n'};
  uint32_t tick = 0;
  while (1)
  {
    GPIOA->ODR = GPIOA->ODR|(1<<5);
    delay_ms(1000);
    // UART2_SendChar('C');
    USART2_Send_DMA(data, 5);
    GPIOA->ODR = GPIOA->ODR &~(1<<5);
    delay_ms(1000);
  }
}

void UART2_SendChar(char c) {
  // Warten, bis das Übertragungsdatenregister leer ist (TXE-Flag gesetzt)
  while (!(USART2->SR & USART_SR_TXE)); 

  // Zeichen in das Datenregister schreiben
  USART2->DR = (uint8_t)c;

  // Optional: Warten, bis das Zeichen vollständig gesendet wurde (TC-Flag gesetzt)
  while (!(USART2->SR & USART_SR_TC)); 
}


/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void USART2_UART_Init(uint32_t baudRate)
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
  USART2->CR3 = 0UL;
  USART2->CR3 |= USART_CR3_DMAT; // enables the DMA trasmition
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

static void DMA_Init_USART2_TX(void){
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
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void GPIO_Init(void)
{
  // GPIOA für die LED initialisieren (PA5)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Takt für GPIOA aktivieren

  GPIOA->MODER &= ~(3UL << (5 * 2));  // Bits löschen
  GPIOA->MODER |= (1UL << (5 * 2));  // PA5 als Ausgang

  GPIOA->OTYPER &= ~(1UL << 5);  // PA5 als Push-Pull
  GPIOA->OSPEEDR |= (3UL << (5 * 2));  // High Speed für PA5
  GPIOA->PUPDR &= ~(3UL << (5 * 2));  // Kein Pull-Up oder Pull-Down für PA5
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
