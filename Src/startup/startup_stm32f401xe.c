#include <stdint.h>
// #include "system_stm32f4xx.h"
// #include "stm32f4xx.h"

/* start address for the initialization values of the .data section. 
defined in linker script */
extern uint32_t  _sidata;/* start address for the .data section. defined in linker script */  
extern uint32_t  _sdata;/* end address for the .data section. defined in linker script */
extern uint32_t  _edata;/* start address for the .bss section. defined in linker script */
extern uint32_t  _sbss;/* end address for the .bss section. defined in linker script */
extern uint32_t  _ebss;/* stack used for SystemInit_ExtMemCtl; always internal RAM used */
extern uint32_t  _estack; // top of stack

// static consturctor funciton of lib c
extern void __libc_init_array();
// main function
extern int main();
extern void SystemInit(void);
// Reset handler (entry point after reset)
void Reset_Handler(){
    uint32_t *src, *dest;
    // Copy the data segment initializers from flash to SRAM
    src = &_sidata;
    for (dest = &_sdata; dest < &_edata;) {
        *dest++ = *src++;
    }
    // Zero fill the .bss segment
    for (dest = &_sbss; dest < &_ebss;) {
        *dest++ = 0;
    }
    //uint32_t *dataInit = &_sidata;
    //uint32_t *data = &_sdata;
    //
    //// Copy the data segment initializers from flash to SRAM
    //while(dataInit<&_edata){
    //    *data++ = *dataInit++;
    //}
    //
    //// Init bss segment to zero
    //uint32_t *bss = &_sbss;
    //while(bss<&_ebss){
    //    *bss++ = 0;
    //}
    // call system initalisation (clock setup etc)
    SystemInit();
    // Call C++ constructors for static objects, if any
    __libc_init_array();
    // Enter main
    main();
    while(1);
}

// Default handler for unimplemented interrupts
void Default_Handler(void) {
    while(1);
}

// Goes ther if divide by zero or dereferencing a  null ponter
//void HardFault_Handler(void){
//    while (1);
//}

#ifndef __weak
    #define __weak   __attribute__((weak))
#endif /* __weak */

__weak void NMI_Handler(void)           {Default_Handler();}
__weak void MemManage_Handler(void)     {Default_Handler();}
__weak void HardFault_Handler(void)     {Default_Handler();}
__weak void BusFault_Handler(void)      {Default_Handler();}
__weak void UsageFault_Handler(void)    {Default_Handler();}
__weak void SVC_Handler(void)           {Default_Handler();}
__weak void DebugMon_Handler(void)      {Default_Handler();}
__weak void PendSV_Handler(void)        {Default_Handler();}
__weak void SysTick_Handler(void)       {Default_Handler();}
  
  /* External Interrupts */
__weak void WWDG_IRQHandler(void)              {Default_Handler();}                                  
__weak void PVD_IRQHandler(void)               {Default_Handler();}                     
__weak void TAMP_STAMP_IRQHandler(void)        {Default_Handler();}                 
__weak void RTC_WKUP_IRQHandler(void)          {Default_Handler();}                    
__weak void FLASH_IRQHandler(void)             {Default_Handler();}                                    
__weak void RCC_IRQHandler(void)               {Default_Handler();}                                      
__weak void EXTI0_IRQHandler(void)             {Default_Handler();}                  
__weak void EXTI1_IRQHandler(void)             {Default_Handler();}                    
__weak void EXTI2_IRQHandler(void)             {Default_Handler();}                    
__weak void EXTI3_IRQHandler(void)             {Default_Handler();}                    
__weak void EXTI4_IRQHandler(void)             {Default_Handler();}                    
__weak void DMA1_Stream0_IRQHandler(void)      {Default_Handler();}            
__weak void DMA1_Stream1_IRQHandler(void)      {Default_Handler();}             
__weak void DMA1_Stream2_IRQHandler(void)      {Default_Handler();}             
__weak void DMA1_Stream3_IRQHandler(void)      {Default_Handler();}             
__weak void DMA1_Stream4_IRQHandler(void)      {Default_Handler();}             
__weak void DMA1_Stream5_IRQHandler(void)      {Default_Handler();}             
__weak void DMA1_Stream6_IRQHandler(void)      {Default_Handler();}             
__weak void ADC_IRQHandler(void)               {Default_Handler();}             
__weak void EXTI9_5_IRQHandler(void)           {Default_Handler();}                    
__weak void TIM1_BRK_TIM9_IRQHandler(void)     {Default_Handler();}     
__weak void TIM1_UP_TIM10_IRQHandler(void)     {Default_Handler();}
__weak void TIM1_TRG_COM_TIM11_IRQHandler(void){Default_Handler();}
__weak void TIM1_CC_IRQHandler(void)           {Default_Handler();}                    
__weak void TIM2_IRQHandler(void)              {Default_Handler();}             
__weak void TIM3_IRQHandler(void)              {Default_Handler();}             
__weak void TIM4_IRQHandler(void)              {Default_Handler();}             
__weak void I2C1_EV_IRQHandler(void)           {Default_Handler();}                    
__weak void I2C1_ER_IRQHandler(void)           {Default_Handler();}                    
__weak void I2C2_EV_IRQHandler(void)           {Default_Handler();}                    
__weak void I2C2_ER_IRQHandler(void)           {Default_Handler();}                      
__weak void SPI1_IRQHandler(void)              {Default_Handler();}   
__weak void SPI2_IRQHandler(void)              {Default_Handler();}             
__weak void USART1_IRQHandler(void)            {Default_Handler();}             
__weak void USART2_IRQHandler(void)            {Default_Handler();}             
__weak void EXTI15_10_IRQHandler(void)         {Default_Handler();}                    
__weak void RTC_Alarm_IRQHandler(void)         {Default_Handler();}                    
__weak void OTG_FS_WKUP_IRQHandler(void)       {Default_Handler();}                        
__weak void DMA1_Stream7_IRQHandler(void)      {Default_Handler();}                    
__weak void SDIO_IRQHandler(void)              {Default_Handler();}             
__weak void TIM5_IRQHandler(void)              {Default_Handler();}             
__weak void SPI3_IRQHandler(void)              {Default_Handler();}             
__weak void DMA2_Stream0_IRQHandler(void)      {Default_Handler();}             
__weak void DMA2_Stream1_IRQHandler(void)      {Default_Handler();}             
__weak void DMA2_Stream2_IRQHandler(void)      {Default_Handler();}             
__weak void DMA2_Stream3_IRQHandler(void)      {Default_Handler();}             
__weak void DMA2_Stream4_IRQHandler(void)      {Default_Handler();}             
__weak void OTG_FS_IRQHandler(void)            {Default_Handler();}             
__weak void DMA2_Stream5_IRQHandler(void)      {Default_Handler();}             
__weak void DMA2_Stream6_IRQHandler(void)      {Default_Handler();}             
__weak void DMA2_Stream7_IRQHandler(void)      {Default_Handler();}             
__weak void USART6_IRQHandler(void)            {Default_Handler();}              
__weak void I2C3_EV_IRQHandler(void)           {Default_Handler();}                    
__weak void I2C3_ER_IRQHandler(void)           {Default_Handler();}                    
__weak void FPU_IRQHandler(void)               {Default_Handler();}
__weak void SPI4_IRQHandler(void)              {Default_Handler();}

// interupt vector table
__attribute__((section(".isr_vector")))
const void (*g_pfnVectors[])(void) = {
    (const void (*)(void))&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    SVC_Handler,
    0,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
  
  /* External Interrupts */
    WWDG_IRQHandler,                   /* Window WatchDog              */                                        
    PVD_IRQHandler,                    /* PVD through EXTI Line detection */                        
    TAMP_STAMP_IRQHandler,             /* Tamper and TimeStamps through the EXTI line */            
    RTC_WKUP_IRQHandler,               /* RTC Wakeup through the EXTI line */                      
    FLASH_IRQHandler,                  /* FLASH                        */                                          
    RCC_IRQHandler,                    /* RCC                          */                                            
    EXTI0_IRQHandler,                  /* EXTI Line0                   */                        
    EXTI1_IRQHandler,                  /* EXTI Line1                   */                          
    EXTI2_IRQHandler,                  /* EXTI Line2                   */                          
    EXTI3_IRQHandler,                  /* EXTI Line3                   */                          
    EXTI4_IRQHandler,                  /* EXTI Line4                   */                          
    DMA1_Stream0_IRQHandler,           /* DMA1 Stream 0                */                  
    DMA1_Stream1_IRQHandler,           /* DMA1 Stream 1                */                   
    DMA1_Stream2_IRQHandler,           /* DMA1 Stream 2                */                   
    DMA1_Stream3_IRQHandler,           /* DMA1 Stream 3                */                   
    DMA1_Stream4_IRQHandler,           /* DMA1 Stream 4                */                   
    DMA1_Stream5_IRQHandler,           /* DMA1 Stream 5                */                   
    DMA1_Stream6_IRQHandler,           /* DMA1 Stream 6                */                   
    ADC_IRQHandler,                    /* ADC1, ADC2 and ADC3s         */                   
    0,               				  /* Reserved                      */                         
    0,              					  /* Reserved                     */                          
    0,                                 /* Reserved                     */                          
    0,                                 /* Reserved                     */                          
    EXTI9_5_IRQHandler,                /* External Line[9:5]s          */                          
    TIM1_BRK_TIM9_IRQHandler,          /* TIM1 Break and TIM9          */         
    TIM1_UP_TIM10_IRQHandler,          /* TIM1 Update and TIM10        */         
    TIM1_TRG_COM_TIM11_IRQHandler,     /* TIM1 Trigger and Commutation and TIM11 */
    TIM1_CC_IRQHandler,                /* TIM1 Capture Compare         */                          
    TIM2_IRQHandler,                   /* TIM2                         */                   
    TIM3_IRQHandler,                   /* TIM3                         */                   
    TIM4_IRQHandler,                   /* TIM4                         */                   
    I2C1_EV_IRQHandler,                /* I2C1 Event                   */                          
    I2C1_ER_IRQHandler,                /* I2C1 Error                   */                          
    I2C2_EV_IRQHandler,                /* I2C2 Event                   */                          
    I2C2_ER_IRQHandler,                /* I2C2 Error                   */                            
    SPI1_IRQHandler,                   /* SPI1                         */                   
    SPI2_IRQHandler,                   /* SPI2                         */                   
    USART1_IRQHandler,                 /* USART1                       */                   
    USART2_IRQHandler,                 /* USART2                       */                   
    0,               				  /* Reserved                       */                   
    EXTI15_10_IRQHandler,              /* External Line[15:10]s        */                          
    RTC_Alarm_IRQHandler,              /* RTC Alarm (A and B) through EXTI Line */                 
    OTG_FS_WKUP_IRQHandler,            /* USB OTG FS Wakeup through EXTI line */                       
    0,                                 /* Reserved     				  */         
    0,                                 /* Reserved       			  */         
    0,                                 /* Reserved 					  */
    0,                                 /* Reserved                     */                          
    DMA1_Stream7_IRQHandler,           /* DMA1 Stream7                 */                          
    0,                                 /* Reserved                     */                   
    SDIO_IRQHandler,                   /* SDIO                         */                   
    TIM5_IRQHandler,                   /* TIM5                         */                   
    SPI3_IRQHandler,                   /* SPI3                         */                   
    0,                                 /* Reserved                     */                   
    0,                                 /* Reserved                     */                   
    0,                                 /* Reserved                     */                   
    0,                                 /* Reserved                     */
    DMA2_Stream0_IRQHandler,           /* DMA2 Stream 0                */                   
    DMA2_Stream1_IRQHandler,           /* DMA2 Stream 1                */                   
    DMA2_Stream2_IRQHandler,           /* DMA2 Stream 2                */                   
    DMA2_Stream3_IRQHandler,           /* DMA2 Stream 3                */                   
    DMA2_Stream4_IRQHandler,           /* DMA2 Stream 4                */                   
    0,                    			  /* Reserved                     */                   
    0,              					  /* Reserved                     */                     
    0,              					  /* Reserved                     */                          
    0,             					  /* Reserved                     */                          
    0,              					  /* Reserved                     */                          
    0,              					  /* Reserved                     */                          
    OTG_FS_IRQHandler,                 /* USB OTG FS                   */                   
    DMA2_Stream5_IRQHandler,           /* DMA2 Stream 5                */                   
    DMA2_Stream6_IRQHandler,           /* DMA2 Stream 6                */                   
    DMA2_Stream7_IRQHandler,           /* DMA2 Stream 7                */                   
    USART6_IRQHandler,                 /* USART6                       */                    
    I2C3_EV_IRQHandler,                /* I2C3 event                   */                          
    I2C3_ER_IRQHandler,                /* I2C3 error                   */                          
    0,                                 /* Reserved                     */                   
    0,                                 /* Reserved                     */                   
    0,                                 /* Reserved                     */                         
    0,                                 /* Reserved                     */                   
    0,                                 /* Reserved                     */                   
    0,                                 /* Reserved                     */                   
    0,                                 /* Reserved                     */
    FPU_IRQHandler,                    /* FPU                          */
    0,                                 /* Reserved                     */                   
    0,                                 /* Reserved                     */
    SPI4_IRQHandler  
};