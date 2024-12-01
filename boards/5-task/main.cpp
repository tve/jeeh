// Fast clock, console w/ DMA @ 2 Mbaud, and tasks with periodic events.
// All board details are in "defs.h", using settings from "platformio.ini".

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Pin led (LED, "P");

uart::Poll<UART_NAME.ADDR> console (ena::UART_NAME, UART_FREQ);

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1 || fd == 2)
        console.transfer(true, (uint8_t*) buf, len);
    return len;
}

void initBoard () {
    fastClock();
    cycles::init();
    console.init(UART_PINS, 2'000'000);

    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}

Ticker ticker;
TICKER_TRIGGER(ticker)

struct Blinker : Task {
    enum TAG { START, TICK };

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ticker.periodic(500, TICK);
                break;
            case TICK:
                led.toggle();
                logf("%d ms", cycles::millis());
                break;
            default:
                fail();
        }
        return out;
    }
};

Blinker blinker;

int main () {
    initBoard();

    // init all tasks in decreasing priority
    ticker.init();
    blinker.init();

    while (true)
        asm ("wfi");
}
