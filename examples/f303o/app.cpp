// Main application code, combines all sorts of time-related functionality.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Ticker ticker;
TICKER_INSTALL(ticker)

struct Blinker : Worker {
    enum TAG { START, TICK };

    bool enable =false;

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ticker.periodic(250, TICK);
                break;
            case TICK:
                if (enable)
                    led.toggle();
                break;
            default:
                fail();
        }
        return out;
    }
};

Blinker blinker;

struct Shell : Worker {
    enum TAG { START, TTYIN };

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                console.read(0, { wId, TTYIN });
                break;
            case TTYIN:
                logf("%d: '%c'", in.eVal, *console.rxPtr);
                switch (*console.rxPtr) {
                    case 'l': blinker.enable = !blinker.enable; break;
                }
                console.read(1, { wId, TTYIN });
                break;
            default:
                fail();
        }
        return out;
    }
};

Shell shell;

int main () {
    initBoard();

    // start workers, in decreasing priority
    ticker.init();
    blinker.init();
    shell.init();

    Worker::send({ ticker.wId, ticker.RATE, 1 });
    Worker::send({ blinker.wId, blinker.START });
    Worker::send({ shell.wId, shell.START });

    while (true)
        asm ("wfi");
}
