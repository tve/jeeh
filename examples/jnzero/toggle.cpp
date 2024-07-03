// Read out the BME280 sensor, alternating between I2C and SPI.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/bme280.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    Pin power ("A1");
    power.mode("P");

    i2c::Dev dev1 { i2cBus, 0x76 };

    constexpr auto ALT_PINS = "A7:H0,A6,A5,A4:HP";
    spi::Dev dev2 { spiBus };
    spiBus.rate = 100;

    bool useI2c = true;
    while (true) {
useI2c = true;
        int32_t t;
        uint32_t p, h;

        auto readSensor = [&](auto dev) {
            BME280 bme280 (dev);
            bme280.init();
            sys::wait(5);
            bme280.getReading(t, p, h);
            bme280.deinit();
        };

        power = 1;
        sys::wait(3);
        if (useI2c) {
            i2cBus.init(I2C_PINS, i2cTiming(400));
            readSensor(dev1);
            i2cBus.deinit();
        } else {
            spiBus.init(ALT_PINS, 10'000);
            readSensor(dev2);
            spiBus.deinit();
        }
        power = 0;

        logf("%s: %d.%02d °C, %d.%04d hPa, %d.%03d %%", useI2c ? "I2C" : "SPI",
                t / 100, t % 100, p / 10000, p % 10000, h / 1000, h % 1000);

        led = 0; // on
        sys::wait(100);
        led = 1;
        sys::wait(400);

        useI2c = !useI2c;
    }
}
