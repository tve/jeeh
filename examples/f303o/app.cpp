// Main application code, combines all sorts of time-related functionality.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Ticker ticker;
TICKER_INSTALL(ticker)

struct Shell : Worker {
    enum TAG { START, TTYIN };

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                console.read(0, { wId, TTYIN });
                break;
            case TTYIN:
                logf("%d: '%c'", in.eVal, *(char*) console.rxPtr);
                console.read(in.eVal, { wId, TTYIN });
                break;
            default:
                fail();
        }
        return out;
    }
};

int main () {
    initBoard();

    Shell shell;

    // start workers, in decreasing priority
    auto tkId = ticker.init();
    auto shId = shell.init();

    Worker::send({ tkId, ticker.RATE, 1 });
    Worker::send({ shId, shell.START });

    while (true)
        asm ("wfi");
}
