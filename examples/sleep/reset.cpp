// Try out the alarm wakeup capabilities of the RTC.

#include <jee.h>
#include "defs.h"
using namespace jeeh;

int main () {
    rtc::reset();
    rtc::init();
    rtc::set({ 1, 2, 3, 11, 22, 33 });

    constexpr Pin led (LED);
    led.mode("P");

    while (true) {
        led = 1;
        sys::wait(100);
        led = 0;
        sys::wait(400);
    }
}
