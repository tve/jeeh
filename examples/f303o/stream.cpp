// Capture and decode DCF77 pulses.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Ticker ticker;
TICKER_INSTALL(ticker)

struct Stream : Worker {
    enum TAG { START, TICK, SENT };

    uint16_t bits, repeat =0, prev =0;
    uint8_t count =0;
    char buf [20];

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ticker.periodic(1, TICK);
                break;
            case TICK:
                led = +dcfDat;
                bits >>= 1;
                bits |= led<<15;
                if (++count >= 16) {
                    count = 0;
                    if (bits == prev && repeat < 99)
                        ++repeat;
                    else {
                        auto n = snprintf(buf, sizeof buf, "%2d %d\n",
                                            repeat, (int16_t) prev);
                        console.write(buf, n, { wId, SENT });
                        repeat = 1;
                        prev = bits;
                    }
                }
                break;
            case SENT:
                break;
            default:
                fail();
        }
        return out;
    }
};

int main () {
    initBoard();
    dcfVcc = 1;

    Stream stream;
    auto tId = ticker.init();
    auto sId = stream.init();

    Worker::send({ tId, ticker.RATE, 1 });
    Worker::send({ sId, stream.START });

    while (true)
        asm ("wfi");
}
