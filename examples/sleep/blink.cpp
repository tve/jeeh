// Blink the LED, with 5s shutdowns in between.

#include <jee.h>
#include "defs.h"
using namespace jeeh;

int main () {
    slowClock(false);

    Pin led (LED);
    led.mode("P");

    rtc::init();

    while (true) {
        led = 1;
        rtc::deepSleep(10, 2);
        led = 0;
        rtc::deepSleep(5000, 4);
    }
}

void jeeh::fail(void const*, char const*, int) { while (true) {} }
