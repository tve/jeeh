// Access some built-in and attached peripherals via the GPIO pins.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    { // I2C bus PH5+PH4
        I2cGpio i2c;
        printf("SCL=PH5, SDA=PH4\n");
        i2c.init("H5,H4");
        i2c.detect(); // look for audio codec
    }
    { // I2C bus PH8+PH7
        I2cGpio i2c;
        printf("SCL=PH8, SDA=PH7\n");
        i2c.init("H8,H7");
        i2c.detect(); // look for audio codec
    }
    { // SPI flash memory
        SpiGpio spi;
        SpiFlash spif (spi);

        spi.init("B15,B14,B13,B12");
        auto t = cycles::count();
        int id = spif.devId();
        t = cycles::count() - t;
        printf("spi: id %06x, %d kB, %d cycles\n", id, spif.size(), t);
        spi.deinit();
    }

    while (true) {
        ledL.toggle();  // red, left
        sys::wait(500);
        ledR.toggle();  // red, right
        ledC.toggle();  // red, on CPU board
        ledB.toggle();  // lcd backlight
    }
}
