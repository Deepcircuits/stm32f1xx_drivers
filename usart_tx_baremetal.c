#include "stm32f1xx.h"
#include <stdint.h>

void delay(volatile uint32_t d)
{
    while(d--);
}

void uart1_tx_init(void)
{
    /* 1. Enable clocks */
    RCC->APB2ENR |= (1 << 2);   // GPIOA clock
    RCC->APB2ENR |= (1 << 14);  // USART1 clock

    /* 2. Configure PA9 as AF Push-Pull, 50 MHz */
    GPIOA->CRH &= ~(0xF << 4);      // clear CNF9 + MODE9
    GPIOA->CRH |=  (0xB << 4);      // MODE9=11 (50MHz), CNF9=10 (AF-PP)

    /* 3. USART1 configuration */
    USART1->CR1 = 0;                // clear CR1
    USART1->CR2 = 0;
    USART1->CR3 = 0;

    /*
     * Baud = 9600
     * Clock = 8 MHz
     * USARTDIV = 8000000 / (16 × 9600) ≈ 52.083
     * Mantissa = 52
     * Fraction = 1
     */
    USART1->BRR = (52 << 4) | 1;

    USART1->CR1 |= (1 << 3);   // TE = 1 (TX enable)
    USART1->CR1 |= (1 << 13);  // UE = 1 (USART enable)
}

void uart1_tx_char(char c)
{
    while(!(USART1->SR & (1 << 7)));  // wait TXE
    USART1->DR = c;
}

void uart1_tx_string(char *s)
{
    while(*s)
    {
        uart1_tx_char(*s++);
    }
}

int main(void)
{
    uart1_tx_init();

    while(1)
    {
        uart1_tx_string("HELLO WORLD FROM STM32\r\n");
        delay(800000);
    }
}
