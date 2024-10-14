// Minimal LED blinker, using a busy loop as delay.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    led.mode("P");  // push-pull output

    while (true) {
        led.toggle();
        sys::wait(250);
    }
}
