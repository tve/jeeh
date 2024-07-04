// Read out the BME280 sensor, alternating between I2C and SPI.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/bme280.h>
using namespace jeeh;
#include "defs.h"

template< typename T >
void readSensor (T& bme, bool f, int32_t* tph) {
    bme.init(f);
    bme.start();
    sys::wait(10);
    bme.getReading(tph);
    //bme.deinit();
};

int main () {
    initBoard();

    i2c::Dev dev1 { i2cBus, 0x76 };
    BME280 bme1 (dev1);

    constexpr auto ALT_PINS = "A7:H0,A6,A5,A4:PV";
    spi::Dev dev2 { spiBus };
    BME280 bme2 (dev2);

    Pin power ("A1");
    power.mode("P");

    bool useSpi = false;
    while (true) {
        int32_t tph [3];

        power = 1;
        sys::wait(5);

        if (useSpi) {
            spiBus.init(ALT_PINS, 10'000);
            readSensor(bme2, useSpi, tph);
            spiBus.deinit();
        } else {
            i2cBus.init(I2C_PINS, i2cTiming(1000));
            readSensor(bme1, useSpi, tph);
            i2cBus.deinit();
        }

        power = 0;

        logf("%s: %d.%02d °C, %d.%04d hPa, %d.%03d %%", useSpi ? "SPI" : "I2C",
             tph[0] / 100, tph[0] % 100,
             tph[1] / 10000, tph[1] % 10000,
             tph[2] / 1000, tph[2] % 1000);

        led = 0; // on
        sys::wait(100);
        led = 1;
        sys::wait(400);

        useSpi = !useSpi;
    }
}
