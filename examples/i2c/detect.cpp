// Simple I2C bus scan.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    I2cGpio i2c;
    i2c.init(I2C_PINS, 1);
    i2c.detect();

    while (true) {
        sys::wait(500);
        led.toggle();
    }
}
