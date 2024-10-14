// Simple I2C bus scan.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    i2cBus.init(I2C_PINS, i2cTiming(100));
    i2c::detect(i2cBus);

    logf("  sizeof i2c::Gpio      = %2d b", sizeof (i2c::Gpio));
    logf("  sizeof i2c::Poll      = %2d b", sizeof (i2c::Poll<I2C_NAME.ADDR>));
    logf("  sizeof i2c::Sync      = %2d b", sizeof (i2c::Sync<I2C_TYPE>));
    logf("  sizeof i2c::Call      = %2d b", sizeof (i2c::Call<I2C_TYPE>));
    logf("  sizeof i2c::Dev<...>  = %2d b", sizeof (i2c::Dev<i2c::Gpio>));

    while (true) {
        sys::wait(500);
        if constexpr (strcmp(LED, "B3") != 0)
            led.toggle();
    }
}
