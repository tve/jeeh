// Read out the BME280 sensor via SPI.

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

    constexpr auto ALT_PINS = "A7:H0,A6,A5,A4:HP";
    spiBus.init(ALT_PINS, 10'000);

    spi::Dev dev { spiBus };
    BME280 bme280 (dev);
    bme280.init(true);

    while (true) {
        bme280.start();
        sys::wait(10);
        int32_t tph [3];
        bme280.getReading(tph);

        logf("%d.%02d °C, %d.%04d hPa, %d.%03d %%",
             tph[0] / 100, tph[0] % 100,
             tph[1] / 10000, tph[1] % 10000,
             tph[2] / 1000, tph[2] % 1000);

        led.toggle();
        sys::wait(500);
    }
}
