// Try out the alarm wakeup capabilities of the RTC.

#include <jee.h>
#include "defs.h"
using namespace jeeh;

void LowPower::start (Message& m) {}
void LowPower::finish () {}

int main () {
    slowClock();

    rtc::init();
    rtc::deepSleep(100, 1); // 100 ms STOP 1:   periodic timer
    rtc::alarm(1000, 0);    // 900 ms STOP 0:   alarm
    rtc::deepSleep(100, 2); // 100 ms STOP 2:   periodic timer
    rtc::alarm(1000, 1);    // 900 ms STOP 1:   alarm
    rtc::deepSleep(100, 0); // 100 ms STOP 0:   periodic timer
    rtc::alarm(1000, 2);    // 900 ms STOP 2:   alarm
    rtc::deepSleep(100, 1); // 100 ms STOP 1:   periodic timer
    rtc::alarm(1000, 4);    // 900 ms SHUTDOWN: alarm then RESET

    constexpr Pin led (LED); led.mode("P"); // never reached
    while (true) { led.toggle(); sys::wait(100); }
}
