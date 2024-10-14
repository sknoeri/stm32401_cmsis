/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
  
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

static void GPIO_Init(void);
static void USART2_UART_Init(void);

int main(void)
{

  GPIO_Init();
  USART2_UART_Init();
  while (1)
  {
    GPIOA->ODR = GPIOA->ODR|(1<<5);
    delay_ms(500);
    GPIOA->ODR = GPIOA->ODR &~(1<<5);
    delay_ms(500);
  }
}



/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void USART2_UART_Init(void)
{
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
  // ENables the GPIOA port clkock
  RCC->AHB1LPENR = RCC->AHB1ENR| RCC_AHB1ENR_GPIOAEN;   // Enable the Clock to the GPIOA register
  GPIOA->MODER = GPIOA->MODER| GPIO_MODER_MODE5_0;      // Putes the GPOAPIN5 into ouput mode
  GPIOA->OTYPER = GPIOA->OTYPER &~GPIO_OTYPER_OT5;    // Puts into push-pull mode
  GPIOA->OSPEEDR = GPIOA->OSPEEDR &~GPIO_OSPEEDER_OSPEEDR5_0 &~GPIO_OSPEEDER_OSPEEDR5_1; // Select low speed  optput mode
  GPIOA->PUPDR = GPIOA->PUPDR &~GPIO_PUPDR_PUPDR5_0 &~GPIO_PUPDR_PUPDR5_1; // No pullup or pull down resitsor
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
