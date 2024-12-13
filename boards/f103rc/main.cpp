// Setup for a comms test board, using five built-in serial ports.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#if 1
Dev<uart::Poll<UART1_CONF>> uart1;
Dev<uart::Poll<UART2_CONF>> uart2;
Dev<uart::Poll<UART3_CONF>> uart3;
Dev<uart::Poll<UART4_CONF>> uart4;
#else
Dev<uart::Async<UART1_CONF>> uart1;
Dev<uart::Async<UART2_CONF>> uart2;
Dev<uart::Async<UART3_CONF>> uart3;
Dev<uart::Async<UART4_CONF>> uart4;

UART1_TRIGGER(uart1)
UART2_TRIGGER(uart2)
UART3_TRIGGER(uart3)
UART4_TRIGGER(uart4)
#endif

int main () {
    initBoard();

    uart1.init(2'000'000);
    uart2.init(2'000'000);
    uart3.init(2'000'000);
    uart4.init(2'000'000);

    // output lines of decreasing length, repeating every 64 lines,
    // ...with an average line length of about 50 characters

    int seq = 0;
    while (true) {
        led = ++seq % 4096 < 64;
        logf("%*c %d ms #%d", 64 - seq%64, '/', cycles::millis(), seq);
        cycles::msBusy(100);
    }
}
