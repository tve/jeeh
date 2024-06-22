// SPI Flash test.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard("spi");

    //SpiGpio spi;
    SpiPoll<SPI_NAME.ADDR> spi (ena::SPI_NAME, SPI_FREQ);
    //SpiSync<SPI_TYPE> spi (SPI_CONF);
    //SpiCall<SPI_TYPE> spi (SPI_CONF);

    spi.init(SPI_PINS, 85'000);
    SpiFlash spif (spi);

    uint8_t buf [512];

    cycles::clear();
    spif.erase(0);
    logf("erased in %d ms", cycles::millis());

    for (auto i = 0U; i < sizeof buf; ++i)
        buf[i] = ~i;
    cycles::clear();
    spif.write(0, buf, sizeof buf);
    logf("written in %d us", cycles::micros());

    uint32_t seq = 0;
    while (true) {
        //spif.serNum(buf);
        //logDump(buf, 8);

        cycles::clear();
        spif.read(seq, buf, sizeof buf);
        auto t = cycles::count();
        logDump(buf, 16, "read");

        logf("\t\t\t id %06x, %6d kB, %6d cy, #%d",
                spif.devId(), spif.size(), t, ++seq);

        sys::wait(10'000);
    }
}
