// Read BMP sensor via I2C.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "fcalc.h"

int main () {
    initBoard();

    i2c::Gpio i2cBus;
    i2c::Dev bmp390 { i2cBus, 0x77 };

    i2cBus.init(I2C_PINS, i2cTiming(1000));
    i2c::detect(i2cBus);

    for (auto i = 0U; i < sizeof config; i += 2)
        bmp390.write(config[i], config[i+1]);

    TrimCoeffs tc;
    bmp390.read(0x31, &tc, sizeof tc); // TRIM_PARAMS
    loadParams(tc);

    while (true) {
        sys::wait(1000);

        uint8_t buf [6];
        bmp390.read(0x04, buf, sizeof buf);

        showReading(buf); // in fcalc.h
    }
}
