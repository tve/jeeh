#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Ticker ticker;
TICKER_INSTALL(ticker)

struct Bridge : Task {
    enum TAG { START, RECV };

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                gpsUart.read(0, { wId, RECV });
                break;
            case RECV:
                led1.toggle();
                ttyUart.write(gpsUart.rxPtr, in.eVal);
                gpsUart.read(in.eVal, { wId, RECV });
                break;
            default:
                fail();
        }
        return out;
    }
};

int main () {
    initBoard();

    gpsUart.init(UART1_PINS, 9600);
    gpsUart.wName = "gps-uart";

    Bridge bridge;
    ticker.init();
    bridge.init();

    while (true)
        asm ("wfi");
}
