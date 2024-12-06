// Read BMP sensor via SPI.

#include "../g431k/bmp390.h"

auto& bmp390 = spiBus;

void testBmpS () {
    spiSelect(spiBus, bmpSel);

    bmpVcc = 0;
    cycles::msBusy(5);
    bmpVcc = 1;
    cycles::msBusy(5);

    bmp390.ioRequest(IO_START|IO_WRITE|IO_STOP, config, sizeof config);

    TrimCoeffs tc;
    bmp390.ioRequest(IO_START|IO_WRITE, (uint8_t*) "\xB1.", 2);
    bmp390.ioRequest(IO_READ|IO_STOP, (uint8_t*) &tc, sizeof tc);
    fp.load(tc);

    for (auto i = 0; i < 3; ++i) {
        uint8_t buf [6];
        bmp390.ioRequest(IO_START|IO_WRITE, (uint8_t*) "\x84.", 2);
        bmp390.ioRequest(IO_READ|IO_STOP, buf, sizeof buf);

        showReading(buf); // in fcalc.h
    }

    bmpVcc = 0;
}
