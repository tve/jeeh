#include <jee.h>
#include <jee/spi.h>
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

    SpiGpio spi;
    SpiFlash spif (spi);

#if STM32F723xx
    Pin::config("E2:U,D13"); // pull qspi d2 & d3 high
    spi.init("C9,C10,B2,B6");

    logf("id %06x, %dK", spif.devId(), spif.size());
#endif
}
