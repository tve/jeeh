// Enter ever-deeper low-power modes, with the last one ending in a reset.

#include <jee.h>
#include "defs.h"
using namespace jeeh;

void LowPower::start (Message& m) {}
void LowPower::finish () {}

int main () {
    slowClock(true);            // 4 MHz

    constexpr Pin led (LED);

    rtc::init();

    rtc::deepSleep(500, 0);     // STOP0
    rtc::deepSleep(500, 0);
    slowClock(true);
    rtc::deepSleep(500, 0);

    rtc::deepSleep(500, 1);     // STOP1
    rtc::deepSleep(500, 1);
    slowClock(true);
    rtc::deepSleep(500, 1);

    rtc::deepSleep(500, 2);     // STOP2
    rtc::deepSleep(500, 2);
    slowClock(true);
    rtc::deepSleep(1000, 2);

    rtc::deepSleep(500, 3);     // STANDBY
    while (true) { led.toggle(); sys::wait(100); } // never reached
}
