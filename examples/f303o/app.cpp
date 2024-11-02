// Main application code, combines all sorts of time-related functionality.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

struct Console : Worker {
    enum TAG { START, TTYIN };

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                console.read(0, { wId, TTYIN });
                break;
            case TTYIN:
                logf("%d: '%c'", in.eVal, *(char*) gps.rxPtr);
                console.read(1, { wId, TTYIN });
                break;
            default:
                fail();
        }
        return out;
    }
};

int main () {
    initBoard();
    gps.init(UART1_PINS, 9600);

    Console console;
    auto id = console.init();
    Worker::send({ id, console.START });

    while (true)
        led = +gpsPps;
}
