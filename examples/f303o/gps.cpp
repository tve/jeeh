// Capture and decode MSF60 pulses.

#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/uart.h>
using namespace jeeh;
#include "defs.h"

struct Echo : Worker {
    enum TAG { START, RECV };

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                gps.read(0, { wId, RECV });
                break;
            case RECV:
                led.toggle();
                logf("got %d", in.eVal);
                _write(1, (char*) gps.rxPtr, in.eVal);
                gps.read(in.eVal, { wId, RECV });
                break;
            default:
                fail();
        }
        return out;
    }
};

int main () {
    initBoard();
    gps.init(UART_PINS, 9600);

    Echo echo;
    auto id = echo.init();
    Worker::send({ id, echo.START });

    while (true)
        asm ("wfi");
}
