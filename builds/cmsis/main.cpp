#include <stm32g4xx.h>

static void delayLoop (int n) {
    for (int i = 0; i < n * 3000; ++i)
        asm ("");
}

int main () {
    extern uint32_t g_pfnVectors [];
    SCB->VTOR = (uint32_t) g_pfnVectors; // make sure VTOR is always correct

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE8_Msk) | GPIO_MODER_MODE8_0;

    while (true) {
        GPIOB->ODR |= 1<<8;
        delayLoop(100);
        GPIOB->ODR &= ~(1<<8);
        delayLoop(400);
    }
}
