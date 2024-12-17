// Fast clock, DMA+WFE console @ 2 Mbaud, tasks, and continuous output.
// All board details are in "defs.h", using settings from "platformio.ini".

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Dev<uart::Async<UARTX_CONF>> uartx;
UARTX_TRIGGER(uartx)

struct Streamer : Task {
    enum TAG { START, SENT };

    char buf [90];
    int seq =0;

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                { trace(INIT); }
                [[fallthrough]];
            case SENT: {
                trace(SEND);
                auto n = snprintf(buf, sizeof buf,
                            "+%*c %d ms #%d\n",
                            32 - seq%32, '/', (int) cycles::millis(), seq);
                ++seq;
                uartx.setReply({ tId, SENT });
                { trace(TICKED); uartx.write(buf, n); }
                break;
            }
            default:
                fail();
        }
        return out;
    }
};

int main () {
    initBoard();
    uartx.init(1'000'000);

    Streamer streamer;

    // init all tasks in decreasing priority
    streamer.init();

    while (true)
        asm ("wfi");
}
