// Fast clock, DMA+WFE console @ 2 Mbaud, tasks, and continuous output.
// All board details are in "defs.h", using settings from "platformio.ini".

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Pin led (LED, "P");

uart::Async<UART_TYPE> console (UART_CONF);

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1 || fd == 2)
        ((uart::Poll<UART_NAME.ADDR>&) console).transfer(true, (uint8_t*) buf, len);
    return len;
}

void initBoard () {
    fastClock();
    cycles::init();
    console.init(UART_PINS, 2'000'000);

    //logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}

Ticker ticker;
TICKER_TRIGGER(ticker)

struct Blinker : Task {
    enum TAG { START, TICK };

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ticker.periodic(500, TICK);
                break;
            case TICK:
                led.toggle();
                break;
            default:
                fail();
        }
        return out;
    }
};

Blinker blinker;

struct Streamer : Task {
    enum TAG { START, SENT };

    char buf [80];
    int seq =0;

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
            case SENT: {
                ++seq;
                auto n = snprintf(buf, sizeof buf, "%*c #%d\n",
                                                        64 - seq%64, '/', seq);
                console.write(buf, n, { tId, SENT });
                break;
            }
            default:
                fail();
        }
        return out;
    }
};

Streamer streamer;

int main () {
    initBoard();

    // init all tasks in decreasing priority
    ticker.init();
    streamer.init();
    blinker.init();

    while (true)
        asm ("wfi");
}
