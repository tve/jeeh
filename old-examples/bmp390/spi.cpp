// Read BMP sensor via SPI.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "fcalc.h"

int main () {
    initBoard();

    auto& bmp390 = spiBus;
    bmp390.init(SPI_PINS, 10'000);

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
        sys::wait(1000);

        uint8_t buf [6];
        bmp390.enable();
        bmp390.transfer(true, (uint8_t*) "\x84.", 2);
        bmp390.transfer(false, buf, sizeof buf);
        bmp390.disable();

        showReading(buf); // in fcalc.h
    }
}
