// Fast clock, polled console @ 2 Mbaud with continuous full speed output.
// All board details are in "defs.h", using settings from "platformio.ini".

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Pin led (LED, "P");

constexpr uart::Config uartCfg {
    UART_PINS,                                 // gpio
    UART_NAME.ADDR, ena::UART_NAME, UART_FREQ, // poll
//  DMA1.ADDR, 1-1, 1-1, 2-1, 27, 26,          // sync
//  Irq::DMA1_CH1, Irq::DMA1_CH1, Irq::USART2,
};

Dev<uart::Poll<uartCfg>> console;

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

    // output lines of decreasing length, repeating every 64 lines,
    // ...with an average line length of about 50 characters

    int seq = 0;
    while (true) {
        led = ++seq % 4096 < 64;
        logf("%*c %d ms #%d", 64 - seq%64, '/', cycles::millis(), seq);
    }
}
