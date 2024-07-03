// Read out the BME280 sensor via I2C.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/bme280.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    Pin sensor ("A1");
    sensor.mode("P");
    sensor = 1;
    sys::wait(10);

    i2cBus.init(I2C_PINS, i2cTiming(400));
    i2c::detect(i2cBus);

    i2c::Dev dev { i2cBus, 0x76 };
    BME280 bme280 (dev);
    bme280.init();
    sys::wait(10);
    //logDump(&bme280.tc, sizeof bme280.tc);

    while (true) {
        int32_t t;
        uint32_t p, h;
        bme280.getReading(t, p, h);

        logf("%d.%02d °C, %d.%04d hPa, %d.%03d %%",
             t / 100, t % 100, p / 10000, p % 10000, h / 1000, h % 1000);

        led.toggle();
        sys::wait(500);
    }
}
