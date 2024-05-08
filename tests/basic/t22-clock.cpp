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

    auto dt1 = rtc::getDate();
    logf("1: %d %d", dt1.ss, dt1.ff);
    sys::wait(1);

    Pin led (LED);
    led.mode("P");

    for (auto i = 0; i < 10; ++i) {
        rtc::deepSleep(25, 1);
        led.toggle();
    }

    fastClock();

    auto dt2 = rtc::getDate();
    logf("2: %d %d", dt2.ss, dt2.ff);

    assert(dt1.ff == 0);
    assert(50 <= dt2.ff && dt2.ff <= 60); // these are 256 Hz ticks, not msec
    assert(dt1.ss == dt2.ss);
}
