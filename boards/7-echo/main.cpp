// Echo incoming data, showing the bytes which were received each time.
// All board details are in "defs.h", using settings from "platformio.ini".

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Pin led (LED, "P");

//Dev<uart::Poll<UART_CONF>> console;
Dev<uart::Sync<UART_CONF>> console;

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1 || fd == 2) {
#if 1
        console.write(buf, len);
#else // this can be used while debugging the sync/async drivers and IRQs
        uart::Poll<UART_CONF>& polledConsole = console;
        polledConsole.ioRequest(IO_WRITE, (uint8_t*) buf, len);
#endif
    }
    return len;
}

void initBoard () {
    fastClock();
    cycles::init();
    //console.init(2'000'000); // this is too fast to capture larger chunks
    console.init(115'200);

    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}

int main () {
    initBoard();

    while (true) {
        uint8_t buf [16];
        auto n = console.read(buf, sizeof buf);
        logDump(buf, n);
    }
}
