// Read BMP sensor via SPI.

#include "../g431k/bmp390.h"

auto& bmp390 = spiBus;

void testBmpS () {
    bmpVcc = 0;
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

    for (auto i = 0; i < 3; ++i) {
        uint8_t buf [6];
        bmp390.enable();
        bmp390.transfer(true, (uint8_t*) "\x84.", 2);
        bmp390.transfer(false, buf, sizeof buf);
        bmp390.disable();

        showReading(buf); // in fcalc.h
    }

    bmpVcc = 0;
}
