#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

i2c::Gpio i2cBus;

int main () {
    initBoard();
    i2cBus.init("B7,B6"); // sda scl

    while (true) {
        led.toggle();
        i2c::detect(i2cBus);
        cycles::msBusy(1000);
    }
}
