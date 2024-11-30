// Minimal serial output example, blinking the LED as it loops.
// All board details are in "defs.h", using settings from "platformio.ini".

#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;
#include "defs.h"

void msDelay (int ms) {
#if STM32G0 | STM32L0
    // Cortex M0+ has no cycle counter and cycles::millis won't advance
    // ... but let's still implement a bit of a delay for the LED blinks
    ms *= SystemCoreClock / 5000;
    for (int i = 0; i < ms; ++i)
        asm ("");
#else
    cycles::msBusy(ms);
#endif
}

int main () {
    cycles::init();
    serout::init();

    while (true) {
        led = 1;
        msDelay(100);
        led = 0;
        msDelay(400);

        printf("%u ms\n", (unsigned) cycles::millis());
    }
}
