// Enter ever-deeper low-power modes, with the last one ending in a reset.

#include <jee.h>
#include "defs.h"
using namespace jeeh;

int main () {
    rtc::init();

    Pin led (LED);
    led.mode("P");
    led = 1;
    rtc::deepSleep(10, 2);
    led = 0;

    rtc::deepSleep(1000, 0);
    rtc::deepSleep(1000, 0);
    fastClock(false);
    rtc::deepSleep(1000, 0);

    rtc::deepSleep(1000, 1);
    rtc::deepSleep(1000, 1);
    fastClock(false);
    rtc::deepSleep(1000, 1);

    rtc::deepSleep(1000, 2);
    rtc::deepSleep(1000, 2);
    fastClock(false);
    rtc::deepSleep(2000, 2);

    rtc::deepSleep(1000, 3);
}

void jeeh::fail(void const*, char const*, int) { while (true) {} }
