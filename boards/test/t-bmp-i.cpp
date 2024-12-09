// Read BMP sensor via I2C.

#include "../g431k/bmp390.h"

void testBmpI () {
    i2cBus.select(0x77);

    cycles::msBusy(5);
    bmpVcc = 1;
    cycles::msBusy(5);

    i2cBus.write(config, sizeof config);

    TrimCoeffs tc;
    i2cBus.readRegs(0x31, &tc, sizeof tc); // TRIM_PARAMS
    fp.load(tc);

    for (auto i = 0; i < 3; ++i) {
        cycles::msBusy(5);

        uint8_t buf [6];
        i2cBus.readRegs(0x04, buf, sizeof buf);
        showReading(buf); // in fcalc.h
    }

    bmpVcc = 0;
}
