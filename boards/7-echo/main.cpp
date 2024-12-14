// Fast clock, DMA+WFE console @ 2 Mbaud, tasks, and continuous output.
// All board details are in "defs.h", using settings from "platformio.ini".

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Pin led (LED, "P");

Dev<uart::Sync<UART_CONF>> console;

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1 || fd == 2)
        console.write(buf, len);
    return len;
}

void initBoard () {
    fastClock();
    cycles::init();
    console.init(2'000'000);

    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}

int main () {
    initBoard();

    while (true) {
        uint8_t ch = 0;
        auto n = console.read(&ch, 1);
        logf("11 %d %c", n, ch);
    }
}
