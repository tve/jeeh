// SPI Flash smoke test.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard("spif");

    //SpiGpio spi;
    //SpiPoll<SPI_NAME.ADDR> spi (ena::SPI_NAME, SPI_FREQ);
    SpiSync<SPI_TYPE> spi (SPI_CONF);
    //SpiCall<SPI_TYPE> spi (SPI_CONF);

    spi.init(SPI_PINS, 85'000);
    SpiFlash spif (spi);

    uint8_t buf [512];

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

    uint32_t seq = 0;
    while (true) {
        //spif.serNum(buf);
        //logDump(buf, 8);

        memset(buf, 0xAA, sizeof buf);
        cycles::clear();
        spif.read(seq, buf, sizeof buf);
        auto t = cycles::count();
        logDump(buf, 16);

        logf("\t\t\t\t\t id %06x, %6d kB, %6d cy, #%d",
                spif.info(), spif.size(), t, ++seq);

        sys::wait(5'000);
    }
}
