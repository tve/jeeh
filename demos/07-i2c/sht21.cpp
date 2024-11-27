#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

i2c::Gpio i2cBus;
enum { SHT21 = 0x40 }; // I2C address

// start a measurement, wait until ready, get 3 result bytes
uint16_t measure (uint8_t type, uint8_t delay) {
    i2cBus.start(SHT21*2);
    i2cBus.wrByte(type);
    i2cBus.stop();

    cycles::msBusy(delay);

    uint8_t buf [3];
    i2cBus.start(SHT21*2+1);
    buf[0] = i2cBus.rdByte(false);
    buf[1] = i2cBus.rdByte(false);
    buf[2] = i2cBus.rdByte(true); // last one
    logDump(buf, sizeof buf);

    return (buf[0] << 8) | (buf[1] & ~3); // clear lower 2 bits
}

int main () {
    initBoard();
    i2cBus.init("B7,B6"); // sda scl

    while (true) {
        led.toggle();

        auto tRaw = measure(0xF3, 85); // measure temperature
        auto hRaw = measure(0xF5, 29); // measure humidity
        auto t = (17572 * tRaw >> 16) - 4685; // °C x 100
        auto h = (12500 * hRaw >> 16) - 600; // %RH x 100
        logf("T: %d.%02d C, RH: %d.%02d %%", t/100, t%100, h/100, h%100);

        cycles::msBusy(1000);
    }
}
