#include "crc.h"
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

    template< typename T >
    int32_t read (T r) const {
        uint32_t v = 0;
        return read(r, &v, sizeof (T)) ? v : -1;
        return v;
    }

    template< typename T >
    bool read (T r, void* p, uint8_t n) const {
        return transfer(bus.R1, &r, sizeof (T))
            && transfer(bus.R2, p, n);
    }

    template< typename T >
    bool write (T r, uint8_t v) const {
        return write(r, &v, sizeof (T));
    }

    template< typename T >
    bool write (T r, void const* p, uint8_t n) const {
        return transfer(bus.W1, &r, sizeof (T))
            && transfer(bus.W2, (void*) p, n);
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
