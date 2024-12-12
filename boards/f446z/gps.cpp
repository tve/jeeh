#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Ticker ticker;
TICKER_TRIGGER(ticker)

struct Bridge : Task {
    enum TAG { START, RECV };

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                gpsUart.read(0, { tId, RECV });
                break;
            case RECV:
                led1.toggle();
                ttyUart.write(gpsUart.rxPtr, in.eVal);
                gpsUart.read(in.eVal, { tId, RECV });
                break;
            default:
                fail();
        }
        return out;
    }
};

int main () {
    initBoard();

    gpsUart.init(9600);
    gpsUart.setName("gps-uart");

    Bridge bridge;
    ticker.init();
    bridge.init();

    while (true)
        asm ("wfi");
}
