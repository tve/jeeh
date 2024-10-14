// Enter ever-deeper low-power modes, with the last one ending in a reset.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    slowClock(true);            // 4 MHz

    constexpr Pin led (LED);

    rtc::init();

    rtc::shortSleep(500, sys::STOP0);
    rtc::shortSleep(500, sys::STOP0);
    slowClock(true);
    rtc::shortSleep(500, sys::STOP0);

    rtc::shortSleep(500, sys::STOP1);
    rtc::shortSleep(500, sys::STOP1);
    slowClock(true);
    rtc::shortSleep(500, sys::STOP1);

    rtc::shortSleep(500, sys::STOP2);
    rtc::shortSleep(500, sys::STOP2);
    slowClock(true);
    rtc::shortSleep(1000, sys::STOP2);

    rtc::shortSleep(500, sys::STANDBY);

    fail(); // blink LED
}
