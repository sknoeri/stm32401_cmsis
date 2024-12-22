/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif
#ifndef STM32F401xE
#define STM32F401xE
#endif
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_UART2.h"
static void GPIO_Init(void);
void Error_Handler(void);
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
