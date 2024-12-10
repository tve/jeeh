// Minimal LED blinker, using a busy loop as delay.
// All board details are in "defs.h", using settings from "platformio.ini".

#include <jee.h>
using namespace jeeh;
#include "defs.h"

Pin led (LED, "P");

void delayLoop (int n) {
    n *= SystemCoreClock >> 12;
    while (--n >= 0)
        asm ("");
}

int main () {
    while (true) {
        led = 1;
        delayLoop(100);
        led = 0;
        delayLoop(400);
    }
}
