#include "stm32f4xx.h"

void SystemClock_Config(void);
void delay_ms(uint32_t ms);
void LED_Init(void);
void LED_Blink(void);

int main(void)
{
    // System Clock konfigurieren
    SystemClock_Config();
    
    // LED initialisieren
    LED_Init();

    // Hauptschleife
    while (1)
    {
        LED_Blink();
    }
}

void SystemClock_Config(void)
{
    // Aktivierung von HSI (High-Speed Internal)
    RCC->CR |= RCC_CR_HSION;  // HSI einschalten
    while (!(RCC->CR & RCC_CR_HSIRDY));  // Warten bis HSI stabil ist

    // PLL Konfiguration
    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos)  // PLLM = 16 (Teilt 16 MHz HSI durch 16, um 1 MHz zu erhalten)
                 | (160 << RCC_PLLCFGR_PLLN_Pos) // PLLN = 160 (Multipliziert 1 MHz mit 160, um 160 MHz zu erhalten)
                 | (0 << RCC_PLLCFGR_PLLP_Pos)   // PLLP = 2 (00: PLLP = 2, 01: PLLP = 4, 10: PLLP = 6, 11: PLLP = 8, teilt 160 MHz durch 2, um 80 MHz zu erhalten)
                 | (RCC_PLLCFGR_PLLSRC_HSI)      // PLL Quelle HSI
                 | (4 << RCC_PLLCFGR_PLLQ_Pos);  // PLLQ = 4

    // PLL aktivieren
    RCC->CR |= RCC_CR_PLLON;  // PLL einschalten
    while (!(RCC->CR & RCC_CR_PLLRDY));  // Warten bis PLL stabil ist

    // Flash Speicher Latenz einstellen
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;  // 2 Wait States

    // AHB, APB1 und APB2 Prescaler einstellen
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1    // AHB Prescaler = 1
               | RCC_CFGR_PPRE1_DIV2   // APB1 Prescaler = 2
               | RCC_CFGR_PPRE2_DIV1;  // APB2 Prescaler = 1

    // Umschalten auf PLL als Systemtaktquelle
    RCC->CFGR |= RCC_CFGR_SW_PLL;  // PLL als Systemtaktquelle
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Warten bis PLL als Systemtaktquelle benutzt wird
}

void SystemInit(void)
{
    // FPU aktivieren, falls verwendet (optional)
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));  // Voller Zugriff auf CP10 und CP11

    // Standard-Konfiguration für Systemtakt
    SystemClock_Config();
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

void LED_Init(void)
{
    // GPIOA für die LED initialisieren (PA5)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Takt für GPIOA aktivieren

    GPIOA->MODER &= ~(3UL << (5 * 2));  // Bits löschen
    GPIOA->MODER |= (1UL << (5 * 2));  // PA5 als Ausgang

    GPIOA->OTYPER &= ~(1UL << 5);  // PA5 als Push-Pull
    GPIOA->OSPEEDR |= (3UL << (5 * 2));  // High Speed für PA5
    GPIOA->PUPDR &= ~(3UL << (5 * 2));  // Kein Pull-Up oder Pull-Down für PA5
}

void LED_Blink(void)
{
    GPIOA->ODR ^= (1UL << 5);  // PA5 umschalten (LED ein/aus)
    delay_ms(1000);  // 1 Sekunde warten
}
