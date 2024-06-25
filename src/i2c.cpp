#include "jee.h"
#include "jee/i2c.h"

namespace jeeh {

void I2cGpio::init (char const* desc, uint16_t khz) {
    Pin::config(desc, &sda, 2);
    Pin::config(":OU,", &sda, 2);

    scl = 1;
    sda = 1;

    // this is merely a wild estimate for the countdown needed in hold()
    // values < 100 will override to define a specific countdown instead
    rate = khz < 100 ? khz : SystemCoreClock/khz/200'000 + 1;
}

bool I2cGpio::start (uint8_t addr) const {
    sclLo();
    sclHi();
    sda = 0;
    return wrByte(addr);
}

void I2cGpio::stop () const {
    sda = 0;
    sclHi();
    sda = 1;
    hold();
}

int I2cGpio::rdByte (bool last) const {
    uint8_t data = 0;
    for (auto mask = 0x80; mask != 0; mask >>= 1) {
        sclHi();
        if (sda)
            data |= mask;
        sclLo();
    }
    sda = last;
    sclHi();
    sclLo();
    if (last)
        stop();
    sda = 1;
    return data;
}

bool I2cGpio::wrByte (uint8_t data) const {
    sclLo();
    for (auto mask = 0x80; mask != 0; mask >>= 1) {
        sda = (data & mask) != 0;
        sclHi();
        sclLo();
    }
    sda = 1;
    sclHi();
    hold();
    bool ack = !sda;
    sclLo();
    return ack;
}

} // namespace jeeh
