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

    uint32_t seq = 0;
    while (true) {
        cycles::clear();
        auto id = spif.devId();
        auto t = cycles::count();

        logf("spi: id %06x %6d kB %6d cy #%d", id, spif.size(), t, ++seq);

        uint8_t buf [8];
        spif.serNum(buf);
        logDump(buf, sizeof buf);

        sys::wait(500);
    }
}
