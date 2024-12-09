// Read BMP sensor via SPI.

#include "../g431k/bmp390.h"

auto& bmp390 = spiBus;

void testBmpS () {
    bmp390.select(bmpSel);

    cycles::msBusy(5);
    bmpVcc = 1;
    cycles::msBusy(5);

    bmp390.write(config, sizeof config);

    TrimCoeffs tc;
    bmp390.readRegs16(0xB100, &tc, sizeof tc);
    fp.load(tc);

    for (auto i = 0; i < 3; ++i) {
        cycles::msBusy(5);

        uint8_t buf [6];
        bmp390.readRegs16(0x8400, buf, sizeof buf);
        showReading(buf); // in fcalc.h
    }

    bmpVcc = 0;
}
