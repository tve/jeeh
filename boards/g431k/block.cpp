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
                logf("  msg %d", cycles::millis());
                done = take(out);
                ticker.delay(500, DELAY);
                break;
            case DELAY:
                logf("delay %d", cycles::millis());
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
    enum TAG { START, TICK, REPLY, BLOCK };

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                logf("sender");
                ticker.periodic(2000, TICK);
                break;
            case TICK:
                logf("\n tick %d", cycles::millis());
                led.toggle();
                ticker.delay(250, BLOCK);
                send({ replier.tId, replier.MSG }, { tId, REPLY });
                break;
            case REPLY:
                logf("reply %d", cycles::millis());
                break;
            case BLOCK:
                logf("block %d", cycles::millis());
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
