// Read BMP sensor via I2C.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "bmp390.h"

//i2c::Gpio i2cBus;
//i2c::Poll<I2C_NAME.ADDR> i2cBus (ena::I2C_NAME, I2C_FREQ);
i2c::Sync<I2C_TYPE> i2cBus (I2C_CONF);

int main () {
    initBoard();

    Pin power ("B6"); // Vcc for BMP390
    power.mode("P");
    power = 1;
    cycles::msBusy(5);

    i2cBus.init(I2C_PINS, i2cTiming(1000));
    i2c::Dev bmp390 { i2cBus, 0x77 };

    for (auto i = 0U; i < sizeof config; i += 2)
        bmp390.write(config[i], config[i+1]);

    TrimCoeffs tc;
    bmp390.read(0x31, &tc, sizeof tc); // TRIM_PARAMS
    fp.load(tc);

    while (true) {
        cycles::msBusy(1000);

        uint8_t buf [6];
        bmp390.read(0x04, buf, sizeof buf);

        showReading(buf); // in fcalc.h
    }
}
