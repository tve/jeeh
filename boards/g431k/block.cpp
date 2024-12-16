#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Ticker ticker;
TICKER_TRIGGER(ticker)

struct Replier : Task {
    enum TAG { START, MSG, DELAY };

    Event done;

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                logf("replier");
                break;
            case MSG:
                logf("  msg %d", cycles::millis()/100);
                done = take(out);
                ticker.delay(500, DELAY);
                break;
            case DELAY:
                logf("delay %d", cycles::millis()/100);
                out = take(done);
                break;
            default:
                fail();
        }
        return out;
    }
};

Replier replier;

struct Sender : Task {
    enum TAG { START, TICK, BLOCK };

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                logf("sender");
                ticker.periodic(2000, TICK);
                [[fallthrough]];
            case TICK:
                led = 1;
                ticker.delay(100, BLOCK);
                logf("\n call %d", cycles::millis()/100);
                call({ replier.tId, replier.MSG });
                logf(" back %d", cycles::millis()/100);
                break;
            case BLOCK:
                led = 0;
                logf("block %d", cycles::millis()/100);
                break;
            default:
                fail();
        }
        return out;
    }
};

Sender sender;

int main () {
    initBoard();

    ticker.init();
    replier.init();
    sender.init();

    while (true) {}
}
