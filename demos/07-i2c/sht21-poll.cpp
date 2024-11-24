#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

i2c::Poll<I2C1.ADDR> i2cBus (ena::I2C1, 50);
i2c::Dev sht21 (i2cBus, 0x40);

// start a measurement, wait until ready, get 3 result bytes
uint16_t measure (uint8_t type, uint8_t delay) {
logf("10");
    sht21.transfer(i2cBus.W1, &type, 1);
logf("11");
    sht21.transfer(i2cBus.W2, nullptr, 0);
logf("12");

    cycles::msBusy(delay);

    uint8_t buf [3];
logf("13");
    sht21.transfer(i2cBus.R1, nullptr, 0);
logf("14");
    sht21.transfer(i2cBus.W2, buf, sizeof buf);
logf("15");

    return (buf[0] << 8) | (buf[1] & ~3); // clear lower 2 bits
}

int main () {
    initBoard();
    i2cBus.init("B7:U4,B6", i2cTiming(400)); // sda scl

    while (true) {
        led.toggle();

        auto tRaw = measure(0xF3, 85); // measure temperature
        auto hRaw = measure(0xF5, 29); // measure humidity
        auto t = (17572 * tRaw >> 16) - 4685; // °C x 100
        auto h = (12500 * hRaw >> 16) - 600; // %RH x 100
        logf("T: %d.%02d C, RH: %d.%02d %%", t/100, t%100, h/100, h%100);

        cycles::msBusy(500);
    }
}
