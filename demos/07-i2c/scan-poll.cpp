#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

i2c::Poll<I2C1.ADDR> i2cBus (ena::I2C1, 50);

int main () {
    initBoard();
    i2cBus.init("B7:U4,B6", i2cTiming(400)); // sda scl

    while (true) {
        led.toggle();
        i2c::detect(i2cBus);
        cycles::msBusy(1000);
    }
}
