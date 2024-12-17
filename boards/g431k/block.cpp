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
    enum TAG { START, TICK, T1, T2, T3 };

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ticker.periodic(2000, TICK);
                [[fallthrough]];
            case TICK:
                led = 1;
                ticker.delay(100, T1);
                ticker.delay(200, T2);
                ticker.delay(300, T3);
                logf("\n call %d", cycles::millis()/100);
                call({ replier.tId, replier.MSG });
                logf(" back %d", cycles::millis()/100);
                led = 0;
                break;
            case T1:
                logf("   t1 %d", cycles::millis()/100);
                break;
            case T2:
                logf("   t2 %d", cycles::millis()/100);
                break;
            case T3:
                logf("   t3 %d", cycles::millis()/100);
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
