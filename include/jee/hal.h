#include "cycles.h"
#include "exti.h"

template< typename BUS >
struct BusDev {
    using ID = typename BUS::ID;

    ID const id;
    BUS const& bus;

    BusDev (BUS& b, ID i) : id (i), bus (b) {}

    // general forwarding definitions, any arguments and return type

    template< typename ...A >
    void transfer (A... a) const { return bus.transfer(id, a...); }

    template< typename ...A >
    void read (A... a) { bus.read(id, a...); }

    template< typename ...A >
    void write (A... a) { bus.write(id, a...); }

    // r/w access of a single-byte "register"

    uint32_t read (uint8_t r) const {
        uint32_t v = 0;
        read(r, &v, 1);
        return v;
    }

    void write (uint8_t r, uint8_t v) const {
        write(r, &v, 1);
    }
};

#include "i2c.h"
#include "spi.h"

#include "i2c-dev.h"
#include "spi-dev.h"

#if STM32F1 | STM32F3 | STM32F4 | STM32F7 | STM32G4 | STM32H7
#include "uart-f1347g4h7.h"
#elif STM32L0 | STM32L4 | STM32WL
#include "uart-l04wl.h"
#endif
