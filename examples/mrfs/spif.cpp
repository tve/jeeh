// SPI Flash smoke test.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    spiBus.init(SPI_PINS, 100'000);
    SpiFlash spif (spiBus);

    uint8_t buf [256];

    cycles::clear();
    spif.erase(0);
    logf("erased in %d ms", cycles::millis());

    memset(buf, 0x55, sizeof buf);
    spif.read(0, buf, sizeof buf);
    logDump(buf, 16, "read");

    for (auto i = 0U; i < sizeof buf; ++i)
        buf[i] = ~i;
    cycles::clear();
    spif.write(0, buf, sizeof buf);
    logf("written in %d us", cycles::micros());

    spif.serNum(buf);
    logDump(buf, 8);

    uint32_t seq = 0, mhz = SystemCoreClock / 1'000'000;
    while (true) {
        memset(buf, 0xAA, sizeof buf);
        cycles::clear();
        spif.read(seq, buf, sizeof buf);
        auto t = cycles::count();
        logDump(buf, 16);

        logf("\t\t\t\t\t id %06x, %d kB, %d us, #%d",
                spif.info(), spif.size(), t/mhz, ++seq);

        sys::wait(5'000);
    }
}
