#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/i2c.h>
using namespace jeeh;
#include "defs.h"

i2c::Gpio i2cBus;
//i2c::Poll<I2C_NAME.ADDR> i2cBus (ena::I2C_NAME, I2C_FREQ);

int main () {
    initBoard();

    i2cBus.init(I2C_PINS, i2cTiming(100));
    i2c::detect(i2cBus);

    while (true) {
        led = 1;
        cycles::msBusy(100);
        led = 0;
        cycles::msBusy(900);
    }
}
