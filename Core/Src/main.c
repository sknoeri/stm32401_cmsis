/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
  
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

static void GPIO_Init(void);
static void USART2_UART_Init(uint32_t);

int main(void)
{

  GPIO_Init();
  USART2_UART_Init(115200);
  while (1)
  {
    GPIOA->ODR = GPIOA->ODR|(1<<5);
    delay_ms(1000);
    UART2_SendChar('C');
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
