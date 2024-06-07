/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
  
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART2_UART_Init(void);

int main(void)
{
  SystemClock_Config();
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
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC->CR = RCC->CR | RCC_CR_HSION;
  while(((RCC->CR) & (RCC_CR_HSIRDY))!= SET){ // wait until HSI
    //do nothing
  }
  //Configure PLL
  RCC->PLLCFGR = RCC->PLLCFGR | RCC_PLLCFGR_PLLSRC_HSI           // HSI clsock ssurce
                |RCC_PLLCFGR_PLLM_4| (336<<RCC_PLLCFGR_PLLN_Pos) // M=1/16 and N= 336
                |RCC_PLLCFGR_PLLP_0| (7<<RCC_PLLCFGR_PLLQ_Pos) ;   // P = 1/4 Q = 7 --> 82MHZ
  RCC->CR = RCC->CR | RCC_CR_PLLON_Msk; // enable PLL
  while(((RCC->CR) & (RCC_CR_PLLRDY))!= SET){ // wiat until PLL is enabled
    //do nothing
  }

  // Initializes the CPU, AHB and APB buses clocks
  RCC->CFGR = RCC->CFGR| RCC_CFGR_HPRE_DIV1               // APH divicson 1/1
              |RCC_CFGR_PPRE1_DIV2| RCC_CFGR_PPRE2_DIV1;   // APB1 prescaler 1/2 APB2 prescaler 1/1
  // enables the PLL as system clock
  RCC->CFGR = RCC->CFGR| RCC_CFGR_SW_PLL;
  // waits until the PLL is set as system clock
  while (RCC->CFGR != RCC_CFGR_SWS_PLL)
  {
    // do nothign 
  }

  // Initalizes the systick counter
  SysTick->CTRL = SysTick->CTRL | (1<<SysTick_CTRL_ENABLE_Pos)                     // Enables the SYStick counter
                  |(1<<SysTick_CTRL_TICKINT_Pos)|(1<<SysTick_CTRL_CLKSOURCE_Pos);  // Enables the systick interrup and the sysclock source = AHB procesor clock =84MHz
  SysTick->LOAD = SysTick->LOAD = (84000-1);                                       // Systick counter to 1ms 
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
  /*
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);*/
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
