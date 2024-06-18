#include "cycles.h"
#include "exti.h"

namespace jeeh {

template< typename BUS >
struct BusDev {
    using ID = typename BUS::ID;

    ID const id;
    BUS const& bus;

    BusDev (BUS& b, ID i) : id (i), bus (b) {}

    template< typename ...A >
    void transfer (A... a) const { return bus.transfer(id, a...); }
};

template< typename DEV >
uint32_t readReg (DEV const& d, uint8_t r) {
    uint32_t v = 0;
    readRegs(d, r, &v, 1);
    return v;
}

template< typename DEV >
void readRegs (DEV const& d, uint8_t r, void* p, uint8_t n) {
    d.transfer(d.bus.R1, &r, 1);
    d.transfer(d.bus.R2, p, n);
}

template< typename DEV >
void writeReg (DEV const& d, uint8_t r, uint8_t v) {
    writeRegs(d, r, &v, 1);
}

template< typename DEV >
void writeRegs (DEV const& d, uint8_t r, void const* p, uint8_t n) {
    d.transfer(d.bus.W1, &r, 1);
    d.transfer(d.bus.W2, (void*) p, n);
}

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
