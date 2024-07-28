// Try out the alarm wakeup capabilities of the RTC.

#include <jee.h>
#include "defs.h"
using namespace jeeh;

int main () {
    slowClock();

    rtc::init();
    rtc::shortSleep(100, sys::STOP1); // 100 ms - periodic timer
    rtc::longSleep(1, sys::STOP0);    // 900 ms - alarm
    rtc::shortSleep(100, sys::STOP2); // 100 ms - periodic timer
    rtc::longSleep(1, sys::STOP1);    // 900 ms - alarm
    rtc::shortSleep(100, sys::STOP0); // 100 ms - periodic timer
    rtc::longSleep(1, sys::STOP2);    // 900 ms - alarm
    rtc::shortSleep(100, sys::STOP1); // 100 ms - periodic timer
    rtc::longSleep(1, sys::SHUTDOWN); // 900 ms - alarm then RESET

    fail(); // blink LED
}
