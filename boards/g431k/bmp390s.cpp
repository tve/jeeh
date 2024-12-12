// Read BMP sensor via SPI.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

//Dev<spi::Gpio<SPI_CONF>> spiBus;
//Dev<spi::Poll<SPI_CONF>> spiBus;
Dev<spi::Sync<SPI_CONF>> spiBus;

#include "t-bmp-s.cpp"

int main () {
    initBoard();
    spiBus.init();

    testBmpS();

    while (true) { cycles::msBusy(500); led.toggle(); }
}
