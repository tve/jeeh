// Definitions for STM32 microcontrollers (see also src/stm32.cpp).
// Lines with "CG" control the code-generated parts of this file.

//CG: svd ioregs

enum struct Irq : uint8_t {
    //CG: svd irqs
#if STM32G4
    LPUART1 = LPUART,
#endif
#if STM32L0
    DMA1_Channel2 = DMA1_Channel2_3,
    DMA1_Channel3 = DMA1_Channel2_3,
    DMA1_Channel4 = DMA1_Channel4_7,
    DMA1_Channel5 = DMA1_Channel4_7,
    DMA1_Channel6 = DMA1_Channel4_7,
    DMA1_Channel7 = DMA1_Channel4_7,
#endif
};

enum : uint16_t {
    //CG: svd rccs
};

namespace ena {
    enum : uint16_t {
        //CG: svd enables
#if STM32F3
        GPIOA         = 17 + 8 * AHBENR,
#elif STM32G0
        GPIOA         =  0 + 8 * 0x34,
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
