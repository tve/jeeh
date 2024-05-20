// Try out the alarm wakeup capabilities of the RTC.

#include <jee.h>
#include "defs.h"
using namespace jeeh;

void LowPower::start (Message& m) {}
void LowPower::finish () {}

int main () {
    slowClock();

    rtc::init();
    rtc::alarm(1000, 0); // STOP0
    rtc::alarm(1000, 1); // STOP1
    rtc::alarm(1000, 2); // STOP2
    rtc::alarm(1000, 4); // SHUTDOWN

    constexpr Pin led (LED); led.mode("P"); // never reached
    while (true) { led.toggle(); sys::wait(100); }
}
