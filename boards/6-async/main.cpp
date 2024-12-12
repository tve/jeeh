// Fast clock, DMA+WFE console @ 2 Mbaud, tasks, and continuous output.
// All board details are in "defs.h", using settings from "platformio.ini".

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Pin led (LED, "P");

Dev<uart::Async<UART_CONF>> console;
UART_TRIGGER(console)

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

struct Streamer : Task {
    enum TAG { START, SENT };

    char buf [80];
    int seq =0;

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
            case SENT: {
                ++seq;
                auto n = snprintf(buf, sizeof buf,
                            "%*c %d ms #%d\n",
                            64 - seq%64, '/', (int) cycles::millis(), seq);
                console.setReply({ tId, SENT });
                console.write(buf, n);
                break;
            }
            default:
                fail();
        }
        return out;
    }
};

int main () {
    initBoard();

    Streamer streamer;
    Blinker blinker;

    // init all tasks in decreasing priority
    ticker.init();
    streamer.init();
    blinker.init();

    while (true)
        asm ("wfi");
}
