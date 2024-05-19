// Check the built-in real-time clock and backup registers.

#include <jee.h>
#include "defs.h"
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

#if STM32G431xx
    // there are no OSC32 pins on Nucleo-32's G431KB, must use the 32 kHz LSI
    rtc::init(false);
#else
    rtc::init();
#endif
    rtc::set({ 1, 2, 3, 11, 22, 33 });

    auto dt1 = rtc::todMillis();
    logf("1: %d", dt1);

    Pin led (LED);
    led.mode("P");

    for (auto i = 0; i < 5; ++i) {
        sys::wait(32);
        led.toggle();
    }

    auto dt2 = rtc::todMillis();
    logf("2: %d", dt2);
    itmFlush();

    for (auto i = 0; i < 5; ++i) {
        rtc::deepSleep(32, 1);
        led.toggle();
    }
    fastClock();

    auto dt3 = rtc::todMillis();
    logf("3: %d", dt3);

    auto ms = dt3 - dt1;
    assert(250 <= ms && ms <= 400);
}
