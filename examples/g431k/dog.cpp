// Keep the watchdog alive with an ever-later periodic callback.

#include <jee.h>
#include <jee/cycles.h>
#include <jee/ticker.h>
using namespace jeeh;
#include "defs.h"

Ticker ticker;
TICKER_INSTALL(ticker)

uint32_t start;

struct Kicker : Worker {
    enum TAG { START, TICK };

    uint16_t ms =1500;

    Event process (Event in, Event out, void*) {
        switch (in.eTag) {
            case TICK:
                logf("kick %4d @ %d ms", ms, rtc::getDate().todMillis()-start);
                dog::kick();
                ms += 100;
                [[fallthrough]];
            case START:
                ticker.delay(ms, TICK);
                break;
            default:
                fail();
        }
        return out;
    }
};

int main () {
    initBoard();
    logf("reset %d", dog::resetCause());
    start = rtc::getDate().todMillis();

    dog::init(2);  // will fire approx. every 2 secs
    Kicker kicker;

    auto tkId = ticker.init();
    auto kkId = kicker.init();

    Worker::send({ tkId, ticker.RATE, 1 });
    Worker::send({ kkId, kicker.START });

    int i = 0;
    while (true) {
        logf("  loop %2d @ %d ms", ++i, rtc::getDate().todMillis()-start);
        cycles::msBusy(500);
    }
}
