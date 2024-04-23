#include <jee.h>
#include <jee/cycles.h>
#include <jee/spi.h>
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

#if STM32F723xx
    cycles::init();
    Pin::config("E2:U,D13"); // pull qspi d2 & d3 high

    { // SPI flash interface with optimised inlined calls to the SPI handler
        SpiGpio spi;

        spi.init("C9,C10,B2,B6");
        SpiFlash spif (spi);
        auto t = cycles::count();
        auto id = spif.devId();
        t = cycles::count() - t;
        logf("  direct: id %06x, %dK, %d cycles", id, spif.size(), t);
        spi.deinit();
    }

    { // SPI flash interface with virtual method calls to the SPI handler
        SpiWrap<SpiGpio> spi;

        spi.init("C9,C10,B2,B6");
        SpiFlash spif (spi);
        auto t = cycles::count();
        auto id = spif.devId();
        t = cycles::count() - t;
        logf(" virtual: id %06x, %dK, %d cycles", id, spif.size(), t);
        spi.deinit();
    }
#endif
}
