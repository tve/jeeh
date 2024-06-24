#include "crc.h"
#include "cycles.h"
#include "exti.h"
#include "rng.h"

namespace jeeh {

template< typename I2C >
struct I2cDev {
    I2C& i2c;
    uint8_t id;

    I2cDev (I2C& b, uint8_t i) : i2c (b), id (i) {}

    template< typename ...A >
    auto transfer (A... a) const { return i2c.transfer(id, a...); }

    // one byte address, single-byte data
    int32_t read (uint8_t r) const {
        uint32_t v = 0;
        return read(r, &v, 1) ? v : -1;
        return v;
    }
    bool write (uint8_t r, uint8_t v) const {
        return write(r, &v, 1);
    }

    // one byte address, to/from buffer
    bool read (uint8_t r, void* p, uint8_t n) const {
        return transfer(i2c.R1, &r, 1)
            && transfer(i2c.R2, p, n);
    }
    bool write (uint8_t r, void const* p, uint8_t n) const {
        return transfer(i2c.W1, &r, 1)
            && transfer(i2c.W2, (void*) p, n);
    }

    // two byte address, to/from buffer
    bool read16 (uint16_t r, void* p, uint8_t n) const {
        return transfer(i2c.R1, &r, 2)
            && transfer(i2c.R2, p, n);
    }
    bool write16 (uint16_t r, void const* p, uint8_t n) const {
        return transfer(i2c.W1, &r, 2)
            && transfer(i2c.W2, (void*) p, n);
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
