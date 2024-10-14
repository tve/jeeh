// Blink the on-board LED @ 2 Hz.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();
    led.mode("P");

    while (true) {
        led = 1;
        sys::wait(100);
        led = 0;
        sys::wait(400);
    }
}
