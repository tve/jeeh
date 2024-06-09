#include <jee.h>
#include <jee/hal.h>
#include <jee/spi-flash.h>
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

#if STM32F723xx
    cycles::init();
    Pin::config("E2:U,D13"); // pull qspi d2 & d3 high

    auto mhz = SystemCoreClock / 1'000'000;
    logf("%d MHz", mhz);

    { // SPI flash interface with optimized inlined calls to the SPI handler
        SpiGpio spi;
        SpiFlash spif (spi);

        spi.init("C9,C10,B2,B6");
        auto t = cycles::count();
        int id = spif.devId();
        t = cycles::count() - t;
        logf("  direct: id %06x, %d kB, %d cycles", id, spif.size(), t);
        spi.deinit();
    }

    { // SPI flash interface with optimized final calls to the SPI handler
        SpiWrap<SpiGpio> spi;
        SpiFlash spif (spi); // can optimize, because SpiWrap is final

        spi.init("C9,C10,B2,B6");
        auto t = cycles::count();
        int id = spif.devId();
        t = cycles::count() - t;
        logf(" virtual: id %06x, %d kB, %d cycles", id, spif.size(), t);
        spi.deinit();
    }

    { // SPI flash interface with abstract virtual calls to the SPI handler
        SpiWrap<SpiGpio> spi;
#if 0
        SpiBase& spib = spi;
        SpiFlash spif (spib); // can't optimize, because it's a base class ref
#else
        SpiFlash<SpiBase> spif (spi); // ... same thing, different notation
#endif

        spi.init("C9,C10,B2,B6");
        auto t = cycles::count();
        int id = spif.devId();
        t = cycles::count() - t;
        logf("abstract: id %06x, %d kB, %d cycles", id, spif.size(), t);
        spi.deinit();
    }
#endif
}
