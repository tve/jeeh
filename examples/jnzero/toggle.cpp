// Read out the BME280 sensor, alternating between I2C and SPI.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/bme280.h>
#include <jee/dev/rf69.h>
using namespace jeeh;
#include "defs.h"

template< typename T >
void readSensor (T& bme, bool f, int32_t* tph) {
    bme.init(f);
    bme.start();
    sys::wait(8);
    bme.getReadings(tph);
    //bme.deinit();
};

int main () {
    initBoard();

    spi::Gpio rfmBus;
    rfmBus.init(SPI_PINS, 10'000);
    RF69 rf (rfmBus);
    rf.init(63, 42, 8686); // node 63, group 42, 868.6 MHz
    rf.receive(nullptr, 0);
    rf.sleep();
    rfmBus.deinit();
    rfmBus.nsel.mode("U");

    i2c::Dev dev1 { i2cBus, 0x76 };
    BME280 bme1 (dev1);

    // config SPI1 to use the sensor pins iso the on-board radio module
    constexpr auto ALT_PINS = "A7:UH0,A6,A5,A4:PH";
    spi::Dev dev2 { spiBus };
    BME280 bme2 (dev2);

    Pin power ("A1");
    power.mode("PV");

    bool useSpi = false;
    while (true) {
        int32_t tph [3];

        power = 1;
        sys::wait(2);

        if (useSpi) {
            spiBus.init(ALT_PINS, 10'000);
            readSensor(bme2, useSpi, tph);
            spiBus.deinit();
            Pin::config("A4:U");
        } else {
            i2cBus.init(I2C_PINS, i2cTiming(100));
            readSensor(bme1, useSpi, tph);
            i2cBus.deinit();
        }

        power = 0;

        logf("%s: %d.%02d °C, %d.%04d hPa, %d.%03d %%", useSpi ? "SPI" : "I2C",
             tph[0] / 100, tph[0] % 100,
             tph[1] / 10000, tph[1] % 10000,
             tph[2] / 1000, tph[2] % 1000);

        sys::wait(useSpi ? 25 : 500);

        useSpi = !useSpi;
    }
}
