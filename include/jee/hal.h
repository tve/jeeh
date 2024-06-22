#include "cycles.h"
#include "exti.h"
#include "rng.h"

namespace jeeh {

template< typename BUS >
struct BusDev {
    using ID = typename BUS::ID;

    BUS& bus;
    ID id;

    BusDev (BUS& b, ID i) : bus (b), id (i) {}

    template< typename ...A >
    auto transfer (A... a) const { return bus.transfer(id, a...); }

    int32_t read (uint8_t r) const {
        uint32_t v = 0;
        return read(r, &v, 1) ? v : -1;
        return v;
    }

    bool read (uint8_t r, void* p, uint8_t n) const {
        return transfer(bus.R1, &r, 1) && transfer(bus.R2, p, n);
    }

    bool write (uint8_t r, uint8_t v) const {
        return write(r, &v, 1);
    }

    bool write (uint8_t r, void const* p, uint8_t n) const {
        return transfer(bus.W1, &r, 1) && transfer(bus.W2, (void*) p, n);
    }
};

} // namespace jeeh

#include "i2c.h"
#include "i2c-dev.h"

#include "spi.h"
#include "spi-dev.h"

#if STM32F1 | STM32F3 | STM32F4 | STM32F7 | STM32G4 | STM32H7
#include "uart-f1347g4h7.h"
#elif STM32L0 | STM32L4 | STM32WL
#include "uart-l04wl.h"
#endif
