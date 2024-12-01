// Definitions for STM32 microcontrollers (see also src/stm32.cpp).
// Lines with "CG" control the code-generated parts of this file.

//CG: svd ioregs

enum struct Irq : uint8_t {
    //CG: svd irqs
#if STM32G4
    LPUART1 = LPUART,
#endif
#if STM32G0
    DMA1_Channel1 = DMA_Channel1,
    DMA1_Channel2 = DMA_Channel2_3,
    DMA1_Channel3 = DMA_Channel2_3,
    DMA1_Channel4 = DMA_Channel4_5_6_7,
    DMA1_Channel5 = DMA_Channel4_5_6_7,
    DMA1_Channel6 = DMA_Channel4_5_6_7,
    DMA1_Channel7 = DMA_Channel4_5_6_7,
    I2C1_EV = I2C1,
    I2C1_ER = I2C1,
#elif STM32L0
    DMA1_Channel2 = DMA1_Channel2_3,
    DMA1_Channel3 = DMA1_Channel2_3,
    DMA1_Channel4 = DMA1_Channel4_7,
    DMA1_Channel5 = DMA1_Channel4_7,
    DMA1_Channel6 = DMA1_Channel4_7,
    DMA1_Channel7 = DMA1_Channel4_7,
    I2C1_EV = I2C1,
    I2C1_ER = I2C1,
#endif
};

enum : uint16_t {
    //CG: svd rccs
};

namespace ena {
    enum : uint16_t {
        //CG: svd enables
// missing info, not properly auto-generated from SVD:
#if STM32F3
        GPIOA         = 17 + 8 * AHBENR,
#elif STM32F4 // F413
        UART9         =  6 + 8 * APB2ENR,
        UART10        =  7 + 8 * APB2ENR,
#elif STM32G0
        GPIOA         =  0 + 8 * 0x34,
#elif STM32G4
        DAC1          = 16 + 8 * AHB2ENR,
        DAC2          = 17 + 8 * AHB2ENR,
        DAC3          = 18 + 8 * AHB2ENR,
        DAC4          = 19 + 8 * AHB2ENR,
#elif STM32L0
        GPIOA         =  0 + 8 * 0x2C,
#endif
    };
} // namespace ena

uint32_t fastClock (bool high =true);
uint32_t slowClock (bool high =true);
uint32_t clockChange (uint32_t hz);

void swoInit (uint32_t baud, uint32_t hz =SystemCoreClock);
void swoWrite (void const* ptr =nullptr, size_t len =0);
