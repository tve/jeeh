// Definitions for STM32 microcontrollers (see also src/stm32.cpp).
// Lines with "CG" control the code-generated parts of this file.

//CG: svd ioregs

enum struct Irq : uint8_t {
    //CG: svd irqs
};

enum : uint16_t {
    //CG: svd rccs
};

enum : uint16_t {
    //CG: svd enables
#if STM32G0
    EN_GPIOA         =  0 + 8 * 0x34,
#endif
};

uint32_t fastClock (bool pll =true);
void itmWrite (void const* ptr, uint32_t len);
