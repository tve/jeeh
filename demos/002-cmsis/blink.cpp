#include <stm32f4xx.h>

void delay (int n) {
    for (volatile int i = 0; i < n * 2000; ++i) {}
}

int main () {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~GPIO_MODER_MODE13_Msk;
    GPIOC->MODER |= GPIO_MODER_MODE13_0;
    
    while (1) {
        GPIOC->ODR &= ~GPIO_ODR_OD13;
        delay(100);
        GPIOC->ODR |= GPIO_ODR_OD13;
        delay(400);
    }
}
