#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/i2c.h>
using namespace jeeh;
#include "defs.h"

i2c::Gpio i2cGpio;

int main () {
    initBoard();

    i2cGpio.init(I2C_PINS, i2cTiming(100));
    i2c::detect(i2cGpio);

    while (true) {
        led = 1;
        cycles::msBusy(100);
        led = 0;
        cycles::msBusy(900);
    }
}
