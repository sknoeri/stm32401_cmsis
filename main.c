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

static void I2C_Init(void)
{
  // Enable clock for I2C1
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

  // Configure GPIOB for I2C1 (PB6 -> SCL, PB7 -> SDA)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // Enable clock for GPIOB

  GPIOB->MODER &= ~((3UL << (6 * 2)) | (3UL << (7 * 2)));  // Clear mode bits for PB6 and PB7
  GPIOB->MODER |= (2UL << (6 * 2)) | (2UL << (7 * 2));  // Set alternate function mode for PB6 and PB7

  GPIOB->OTYPER |= (1UL << 6) | (1UL << 7);  // Open-drain for PB6 and PB7
  GPIOB->OSPEEDR |= (3UL << (6 * 2)) | (3UL << (7 * 2));  // High speed for PB6 and PB7
  GPIOB->PUPDR &= ~((3UL << (6 * 2)) | (3UL << (7 * 2)));  // No pull-up, no pull-down for PB6 and PB7

  GPIOB->AFR[0] |= (4UL << (6 * 4)) | (4UL << (7 * 4));  // Set alternate function 4 (I2C1) for PB6 and PB7

  // Configure I2C1
  I2C1->CR1 = 0;  // Clear control register
  I2C1->CR2 = 16;  // Set peripheral clock frequency to 16 MHz
  I2C1->CCR = 80;  // Configure clock control register for 100 kHz I2C clock
  I2C1->TRISE = 17;  // Configure maximum rise time

  I2C1->CR1 |= I2C_CR1_PE;  // Enable I2C1
  // SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0);  // EXTI0 auf PA0 konfigurieren

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
