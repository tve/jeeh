// Read BMP sensor via SPI.

#include "../g431k/bmp390.h"

auto& bmp390 = spiBus;

void testBmpS () {
    bmp390.select(bmpSel);

    bmpVcc = 0;
    cycles::msBusy(5);
    bmpVcc = 1;
    cycles::msBusy(5);

    bmp390.ioRequest(IO_START|IO_WRITE|IO_STOP, config, sizeof config);

    TrimCoeffs tc;
    IoReq req [] = {
        { IO_START|IO_WRITE, 2, (uint8_t*) "\xB1." },
        { IO_READ|IO_STOP, sizeof tc, (uint8_t*) &tc },
    };
    bmp390.ioRequest(req);
    fp.load(tc);

    for (auto i = 0; i < 3; ++i) {
        uint8_t buf [6];
        IoReq req [] = {
            { IO_START|IO_WRITE, 2, (uint8_t*) "\xB4." },
            { IO_READ|IO_STOP, sizeof buf, buf },
        };
        bmp390.ioRequest(req);

        showReading(buf); // in fcalc.h
    }

    bmpVcc = 0;
}
