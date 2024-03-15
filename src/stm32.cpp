#include "jee.h"

#if STM32

namespace jeeh {

#if STM32F1
#include "stm32f1.h"
#elif STM32F3
#include "stm32f3.h"
#elif STM32F4
#include "stm32f4.h"
#elif STM32F7
#include "stm32f7.h"
#elif STM32G0
#include "stm32g0.h"
#elif STM32G4
#include "stm32g4.h"
#elif STM32H7
#include "stm32h7.h"
#elif STM32L0
#include "stm32l0.h"
#elif STM32L4
#include "stm32l4.h"
#endif // STM32??

#if !STM32G0 && !STM32L0 // Cortex M0+ doesn't support ITM

void itmWrite (void const* ptr, size_t len) {
    constexpr IoReg<0xE000'0000> ITM;
    enum { TER=0xE00, TCR=0xE80 };

    if (ITM[TCR](0) && ITM[TER](0)) { // ITM and channel 0 both enabled
        auto pos = (uintptr_t) ptr;
        while (len > 0) {
            while (!ITM[0](0)) {}
            int step = pos % 4 == 0 && len >= 4 ? 4 : 1;
            if (step == 4)
                ITM[0] = *(uint32_t const*) pos;
            else
                ITM.byte(0) = *(uint8_t const*) pos;
            pos += step;
            len -= step;
        }
    }
}

#endif // !STM32L0

} // namespace jeeh

#endif // STM32
