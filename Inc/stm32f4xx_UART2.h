/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32f4XX_UART2
#define __STM32f4XX_UART2

#ifdef __cplusplus
extern "C" {
#endif
#ifndef STM32F401xE
#define STM32F401xE
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include <stdlib.h>

/* Funtion definitons --------------------------------------------------------*/
extern void USART2_UART_Init(uint32_t baudRate, uint8_t UseDMA);
extern void UART2_SendChar(char c);
extern void DMA_Init_USART2_TX(void);
extern void USART2_Send_DMA(char *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* __STM32f4XX_UART2 */