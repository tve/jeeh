// Use external cap on A9 and A10 to generate time-delayed interrupts.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

ExtIrq exti;

int main () {
    initBoard();
    Pin::config("B4:V15"); // EVENTOUT

    Pin rc ("A12");

    auto i = 0;
    while (true) {
        logf("%d", ++i); // uses blocking polled I/O

        sys::wait(100);
        rc.mode(i & 1 ? "U": "D");
        led.toggle();

        Message m {exti.dId, 'A', 12, (uint8_t*) ExtIrq::BOTH};
        sys::call(m);
    }
}

uint8_t jeeh::lowestPower (uint8_t power, uint16_t) {
    Tracer<11> pt;
    return sys::STOP2;
}

void jeeh::resumePower () {
    Tracer<12> pt;
}
