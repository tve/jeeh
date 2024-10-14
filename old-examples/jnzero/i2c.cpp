// Read out the BME280 sensor via I2C.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/bme280.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    Pin power ("A1");
    power.mode("P");
    power = 1;
    sys::wait(5);

    i2cBus.init(I2C_PINS, i2cTiming(1000));
    sys::wait(10);
    //i2c::detect(i2cBus);

    i2c::Dev dev { i2cBus, 0x76 };
    BME280 bme280 (dev);
    bme280.init(false);

    while (true) {
        bme280.start();
        sys::wait(10);
        int32_t tph [3];
        bme280.getReadings(tph);

        logf("%d.%02d °C, %d.%04d hPa, %d.%03d %%",
             tph[0] / 100, tph[0] % 100,
             tph[1] / 10000, tph[1] % 10000,
             tph[2] / 1000, tph[2] % 1000);

        led.toggle();
        sys::wait(500);
    }
}
