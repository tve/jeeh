// Read BMP sensor via I2C.

#include "../g431k/bmp390.h"

void testBmpI () {
    bmpVcc = 0;
    cycles::msBusy(5);
    bmpVcc = 1;
    cycles::msBusy(5);

    i2c::Dev bmp390 { i2cBus, 0x77 };

    for (auto i = 0U; i < sizeof config; i += 2)
        bmp390.write(config[i], config[i+1]);

    TrimCoeffs tc;
    bmp390.read(0x31, &tc, sizeof tc); // TRIM_PARAMS
    fp.load(tc);

    for (auto i = 0; i < 3; ++i) {
        uint8_t buf [6];
        bmp390.read(0x04, buf, sizeof buf);

        showReading(buf); // in fcalc.h
    }

    bmpVcc = 0;
}
