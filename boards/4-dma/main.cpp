// Fast clock, DMA+WFE console @ 2 Mbaud with continuous full speed output.
// All board details are in "defs.h", using settings from "platformio.ini".

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Pin led (LED, "P");

uart::Sync<UART_TYPE> console (UART_CONF);

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1 || fd == 2)
        console.transfer(true, (uint8_t*) buf, len);
    return len;
}

void initBoard () {
    fastClock();
    cycles::init();
    console.init(UART_PINS, 2'000'000);

    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}

int main () {
    initBoard();

    // output lines of decreasing length, repeating every 64 lines,
    // ...with an average line length of about 50 characters

    int seq = 0;
    while (true) {
        led = ++seq % 4096 < 64;
        logf("%*c %d ms #%d", 64 - seq%64, '/', cycles::millis(), seq);
    }
}
