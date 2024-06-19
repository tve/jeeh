#include "cycles.h"
#include "exti.h"

namespace jeeh {

template< typename BUS >
struct BusDev {
    using ID = typename BUS::ID;

    BUS& bus;
    ID id;

    BusDev (BUS& b, ID i) : bus (b), id (i) {}

    template< typename ...A >
    void transfer (A... a) const { return bus.transfer(id, a...); }

    uint32_t readReg (uint8_t r) const {
        uint32_t v = 0;
        readRegs(r, &v, 1);
        return v;
    }

    void readRegs (uint8_t r, void* p, uint8_t n) const {
        transfer(bus.R1, &r, 1);
        transfer(bus.R2, p, n);
    }

    void writeReg (uint8_t r, uint8_t v) const {
        writeRegs(r, &v, 1);
    }

    void writeRegs (uint8_t r, void const* p, uint8_t n) const {
        transfer(bus.W1, &r, 1);
        transfer(bus.W2, (void*) p, n);
    }
};

} // namespace jeeh

#include "i2c.h"
#include "spi.h"

#include "i2c-dev.h"
#include "spi-dev.h"

#if STM32F1 | STM32F3 | STM32F4 | STM32F7 | STM32G4 | STM32H7
#include "uart-f1347g4h7.h"
#elif STM32L0 | STM32L4 | STM32WL
#include "uart-l04wl.h"
#endif
