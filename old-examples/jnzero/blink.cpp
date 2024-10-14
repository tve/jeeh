// Minimal LED blinker.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    Pin led (LED);  // defined in platformio.ini
    led.mode("P");  // push-pull output

    while (true) {
        led = 0; // inverted logic
        sys::wait(100);
        led = 1;
        sys::wait(400);
    }
}
