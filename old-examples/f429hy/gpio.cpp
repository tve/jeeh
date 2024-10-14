// Access some built-in and attached peripherals via the GPIO pins.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    { // I2C bus PH5+PH4
        i2c::Gpio i2cBus;
        printf("SCL=PH5, SDA=PH4\n");
        i2cBus.init("H5,H4");
        i2c::detect(i2cBus); // look for audio codec
    }
    { // I2C bus PH8+PH7
        i2c::Gpio i2cBus;
        printf("SCL=PH8, SDA=PH7\n");
        i2cBus.init("H8,H7");
        i2c::detect(i2cBus); // look for audio codec
    }
    { // SPI flash memory
        spi::Gpio spiBus;
        SpiFlash spif (spiBus);

        spiBus.init("B15,B14,B13,B12");
        auto t = cycles::count();
        int id = spif.info();
        t = cycles::count() - t;
        printf("spi: id %06x, %d kB, %d cycles\n", id, spif.size(), t);
        spiBus.deinit();
    }

    while (true) {
        ledL.toggle();  // red, left
        sys::wait(500);
        ledR.toggle();  // red, right
        ledC.toggle();  // red, on CPU board
        ledB.toggle();  // lcd backlight
    }
}
