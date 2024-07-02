// Simple I2C sensor readout example.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

uint8_t bcd2bin (uint8_t b) { return b - 6 * (b/16); }
uint8_t bin2bcd (uint8_t b) { return b + 6 * (b/10); }

int main () {
    initBoard();
    i2cBus.init(I2C_PINS, 400);
    i2c::Dev ds3231 {i2cBus, 0x68};

    if (0) { // set the date
        DateTime now; // the time this code was compiled, that is ...
        uint8_t buf [] = { bin2bcd(now.ss),
                           bin2bcd(now.mm),
                           bin2bcd(now.hh),
                           1, // day of week, not used
                           bin2bcd(now.dy),
                           bin2bcd(now.mo),
                           bin2bcd(now.yr) };
        ds3231.write(0x00, buf, sizeof buf);
    }

    uint8_t buf [18];
    ds3231.read(0x00, buf, sizeof buf);
    logDump(buf);

    DateTime dt { bcd2bin(buf[6]),
                  bcd2bin(buf[5]),
                  bcd2bin(buf[4]),
                  bcd2bin(buf[2]),
                  bcd2bin(buf[1]),
                  bcd2bin(buf[0]) };

    logf("20%02d/%02d/%02d %02d:%02d:%02d",
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss);

    while (true) {
        sys::wait(250);
        led.toggle();
    }
}
