// Generate time-delayed pin interrupts with an external capacitor on A12.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

ExtIrq exti;

#if 0 // TODO experimental code to determine the overhead of irqDispath
extern "C" void EXTI15_10_IRQHandler () {
    Tracer<9> pt;
    exti.interrupt(0);
    return exti.finish();
}
#endif

int main () {
    initBoard();
    Pin::config("B4:V15"); // EVENTOUT

    Pin rc ("A12"); // this pin has ≈800 nF capacitance tied to ground

    auto i = 0;
    while (true) {
        Tracer<10> pt;
        logf("%d", ++i); // uses blocking polled I/O
        sys::wait(100);

        led.toggle();
        rc.mode(rc ? "D": "U"); // switch to pull-down or pull-up

        // wait for the pin interrupt, some 15..25 ms from now
        Message m {exti.dId, 'A', 12, (uint8_t*) ExtIrq::BOTH};
        sys::call(m);

        rc = +rc;     // force output to same state as currently read
        rc.mode("P"); // ... then enable push-pull mode
    }
}

uint8_t jeeh::lowestPower (uint8_t power, uint16_t) {
    Tracer<11> pt;
    return sys::STOP2;
}

void jeeh::resumePower () {
    Tracer<12> pt;
}
