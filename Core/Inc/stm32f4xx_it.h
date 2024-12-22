#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void delay_ms(uint32_t delay);
uint32_t getTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
