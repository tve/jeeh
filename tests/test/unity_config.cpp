#if 0 // CMSIS version:

#include <stm32g4xx.h>

extern "C" int putchar (int ch) {
    if (USART2->BRR == 0) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
        RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

        GPIOA->AFR[0]  &= ~(0xF << (2*4));
        GPIOA->AFR[0]  |=  (7   << (2*4));
        GPIOA->PUPDR   |=  (1   << (2*2));
        GPIOA->OSPEEDR &= ~(0x3 << (2*2));
        GPIOA->OSPEEDR |=  (2   << (2*2));
        GPIOA->OTYPER  &= ~(0x1 << (2*1));
        GPIOA->MODER   &= ~(0x3 << (2*2));
        GPIOA->MODER   |=  (2   << (2*2));

        for (auto i = SystemCoreClock/25; i > 0; --i) // startup delay
            asm ("");

        USART2->BRR = 16; // 1,000,000 bd @ 16 Mhz, 10,000,000 bd @ 160 MHz
        USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
    }

    while ((USART2->ISR & USART_ISR_TXE) == 0) {}
    USART2->TDR = ch;
    return ch;
}

#else // JeeH version:

#include <jee.h>
using namespace jeeh;

extern "C" int putchar (int ch) {
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, TDR=0x28 };

    if (USART2[BRR] == 0) {
        Pin::config("A2:U7"); // pull-up prevents bad 1st char after reset
        RCC(ena::USART2,1) = 1;
        USART2[BRR] = 16; // 1,000,000 bd @ 16 Mhz, 10,000,000 bd @ 160 MHz
        USART2[CR1] = (1<<3) | (1<<0); // TE UE

        for (auto i = SystemCoreClock/25; i > 0; --i) // startup delay
            asm ("");
    }

    while (!USART2[ISR](7)) {} // TXE
    USART2[TDR] = ch;
    return ch;
}

#endif
