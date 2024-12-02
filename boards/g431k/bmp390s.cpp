// Read BMP sensor via SPI.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "bmp390.h"

//spi::Gpio bmp390;
spi::Poll<SPI_NAME.ADDR> bmp390 (ena::SPI_NAME, SPI_FREQ);
//spi::Sync<SPI_TYPE> bmp390 (SPI_CONF);

int main () {
    initBoard();

    cycles::msBusy(5);
    bmpVcc = 1;
    cycles::msBusy(5);

    bmp390.init(SPI_PINS, 20'000);

    bmp390.enable();
    bmp390.transfer(true, config, sizeof config);
    bmp390.disable();

    TrimCoeffs tc;
    bmp390.enable();
    bmp390.transfer(true, (uint8_t*) "\xB1.", 2);
    bmp390.transfer(false, (uint8_t*) &tc, sizeof tc);
    bmp390.disable();
    fp.load(tc);

    while (true) {
        cycles::msBusy(1000);

        uint8_t buf [6];
        bmp390.enable();
        bmp390.transfer(true, (uint8_t*) "\x84.", 2);
        bmp390.transfer(false, buf, sizeof buf);
        bmp390.disable();

        showReading(buf); // in fcalc.h
    }
}
