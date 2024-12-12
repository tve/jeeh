// I2C example with two different OLED displays.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

//Dev<i2c::Gpio<I2C_CONF>> i2cBus;
//Dev<i2c::Poll<I2C_CONF>> i2cBus;
Dev<i2c::Sync<I2C_CONF>> i2cBus;

#include "t-oled.cpp"

int main () {
    initBoard();
    i2cBus.init(1000);

    testOled();

    while (true) { cycles::msBusy(500); led.toggle(); }
}
