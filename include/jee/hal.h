#include "cycles.h"
#include "exti.h"

template< typename BUS >
struct Dev {
    using ID = typename BUS::ID;

    ID const id;
    BUS& bus;

    Dev (BUS& b, ID i) : id (i), bus (b) {}

    template< typename ...A >
    auto read (A... a) { return bus.read(id, a...); }

    template< typename ...A >
    auto write (A... a) { return bus.write(id, a...); }
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
