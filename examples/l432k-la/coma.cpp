// Examine the control flow of sys::coma.

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
        sys::coma(1, sys::STOP2);
        logf("hello");
    }
}

uint8_t jeeh::lowestPower (uint8_t power, uint16_t) {
    Tracer<11> pt;
    return power;
}

void jeeh::resumePower () {
    Tracer<12> pt;
}
