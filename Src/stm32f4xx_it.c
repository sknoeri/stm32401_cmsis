
#include "stm32f4xx_it.h"

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
static volatile uint32_t tick;

void NMI_Handler(void)
{
  
   while (1)
  {

  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  
  while (1)
  {
    
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
 
  while (1)
  {

  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{

  while (1)
  {
    
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  
  while (1)
  {
    
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{

}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
 
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  tick += 1;
}

uint32_t getTick(void){
  return tick;
}
void delay_ms(uint32_t ms)
{
    // Delay-Funktion basierend auf dem SysTick-Timer
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;  // 1 ms
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    for (uint32_t i = 0; i < ms; i++)
    {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    }

    SysTick->CTRL = 0;  // SysTick Timer deaktivieren
}
/*void delay_ms(uint32_t delay){
  uint32_t t0;
  t0 = getTick();
  while (getTick()-t0<delay){
    //do nothing
  }
  
}*/
/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/


