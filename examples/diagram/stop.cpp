// Examine the control flow of automatic stop mode.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();
    Pin::config("B4:V15"); // EVENTOUT
    asm ("wfi");

    while (true) {
        Tracer<10> pt;
        sys::wait(100);
        logf("hello");
    }
}

uint8_t jeeh::lowestPower (uint8_t power, uint16_t) {
    Tracer<11> pt;
    return sys::STOP2;
}

void jeeh::resumePower () {
    Tracer<12> pt;
}
