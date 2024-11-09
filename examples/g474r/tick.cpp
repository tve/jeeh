#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Ticker ticker;
TICKER_INSTALL(ticker)

struct Blinker : Worker {
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

int main () {
    initBoard();

    Blinker blinker;
    ticker.init();
    blinker.init();

    while (true)
        asm ("wfi");
}
