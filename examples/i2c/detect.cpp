// Simple I2C bus scan.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    i2c.init(I2C_PINS, i2cTiming(100));
    detect(i2c);

    logf("  sizeof I2cGpio      = %2d b", sizeof (I2cGpio));
    logf("  sizeof I2cPoll      = %2d b", sizeof (I2cPoll<I2C_NAME.ADDR>));
    logf("  sizeof I2cSync      = %2d b", sizeof (I2cSync<I2C_TYPE>));
    logf("  sizeof I2cCall      = %2d b", sizeof (I2cCall<I2C_TYPE>));
    logf("  sizeof I2cDev<...>  = %2d b", sizeof (I2cDev<I2cGpio>));

    while (true) {
        sys::wait(500);
        led.toggle();
    }
}
