// Capture and decode DCF77 pulses.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Ticker ticker;
TICKER_INSTALL(ticker)

struct Stream : Worker {
    enum TAG { START, TICK };

    uint16_t bits;
    uint8_t count =0;

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
                    char buf [12];
                    auto len = snprintf(buf, sizeof buf, "%d,", (int16_t) bits);
                    _write(1, buf, len);
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
    dcfVcc = 1;

    Stream stream;
    auto tId = ticker.init();
    auto sId = stream.init();

    Worker::send({ tId, ticker.RATE, 1 });
    Worker::send({ sId, stream.START });

    while (true)
        asm ("wfi");
}
