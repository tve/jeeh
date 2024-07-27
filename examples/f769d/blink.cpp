// Minimal LED blinker, using a busy loop as delay.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    Pin led (LED);  // defined in platformio.ini
    led.mode("P");  // push-pull output

    while (true) {
        led.toggle();

        for (auto i = 0; i < 2'000'000; ++i)
            asm ("");  // prevents getting optimised away
    }
}
