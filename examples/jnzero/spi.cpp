// Read out the BME280 sensor via SPI.

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

    constexpr auto ALT_PINS = "A7:H0,A6,A5,A4:HP";
    spiBus.init(ALT_PINS, 1'000);

    spi::Dev dev { spiBus };
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
