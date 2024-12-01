// Fast clock, console w/ DMA @ 2 Mbaud, and tasks with periodic events.
// All board details are in "defs.h", using settings from "platformio.ini".

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

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
                logf("%d ms", cycles::millis());
                break;
            default:
                fail();
        }
        return out;
    }
};

Blinker blinker;

struct Streamer : Task {
    enum TAG { START };

    char buf [58] = "                                                        /";

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                for (auto seq = 0;; ++seq)
                    logf("%s %d", buf + seq % (sizeof buf - 2), seq);
                break;
            default:
                fail();
        }
        return out;
    }
};

Streamer streamer;

int main () {
    initBoard();

    // the order of these inits defines the (decreasing) task priorities
    ticker.init();
    blinker.init();
    streamer.init();

    while (true)
        asm ("wfi");
}
