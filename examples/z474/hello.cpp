// Init board uwing printf to ID itself, then blink the LED.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard("hello");

    while (true) {
        led.toggle();
        sys::wait(500);
    }
}
