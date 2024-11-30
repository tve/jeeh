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
                ticker.periodic(250, TICK);
                break;
            case TICK:
                led1.toggle();
                if (!led1) {
                    led2.toggle();
                    if (!led2)
                        led3.toggle();
                }
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
