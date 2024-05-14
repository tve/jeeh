// Definitions for STM32 microcontrollers (see also src/stm32.cpp).
// Lines with "CG" control the code-generated parts of this file.

//CG: svd ioregs

enum struct Irq : uint8_t {
    //CG: svd irqs
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

void swoInit (uint32_t baud, uint32_t hz =SystemCoreClock);
void itmWrite (void const* ptr, size_t len);
