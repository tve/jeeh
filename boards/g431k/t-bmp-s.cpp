// Read BMP sensor via SPI.

#include "../g431k/bmp390.h"

void testBmpS () {
    spiBus.select(bmpSel);

    cycles::msBusy(5);
    bmpVcc = 1;
    cycles::msBusy(5);

    spiBus.write(config, sizeof config);

    TrimCoeffs tc;
    spiBus.readRegs16(0xB100, &tc, sizeof tc); // TRIM_PARAMS
    fp.load(tc);

    for (auto i = 0; i < 3; ++i) {
        cycles::msBusy(5);

        uint8_t buf [6];
        spiBus.readRegs16(0x8400, buf, sizeof buf);
        showReading(buf); // in fcalc.h
    }

    bmpVcc = 0;
}
