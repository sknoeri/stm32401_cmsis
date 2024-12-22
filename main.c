/* Includes ------------------------------------------------------------------*/
#include "main.h"
// #include "stm32f4xx_UART2.h"
/* function prototypes -----------------------------------------------*/
int main(void)
{

  GPIO_Init();
  USART2_UART_Init(115200,1);
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
