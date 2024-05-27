// Blink the on-board LED @ 2 Hz (PA8).

#include <jee.h>
#include "defs.h"
using namespace jeeh;

int main () {
    Pin led (LED);
    led.mode("P");

    while (true) {
        led = 1;
        sys::wait(100);
        led = 0;
        sys::wait(400);
    }
}
