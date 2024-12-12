// Drive a 1.44" 128x128 LCD display using polled SPI.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

//Dev<spi::Gpio<SPI_CONF>> spiBus;
//Dev<spi::Poll<SPI_CONF>> spiBus;
Dev<spi::Sync<SPI_CONF>> spiBus;

#include "t-lcd.cpp"

int main () {
    initBoard();
    spiBus.init();

    testLcd();

    while (true) { cycles::msBusy(500); led.toggle(); }
}
