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

    auto ms1 = rtc::getDate().todMillis();
    logf("1: %d", ms1);

    Pin led (LED);
    led.mode("P");

    for (auto i = 0; i < 5; ++i) {
        sys::wait(30);
        led.toggle();
    }

   auto ms2 = rtc::getDate().todMillis();
    logf("2: %d", ms2); swoWrite();

    for (auto i = 0; i < 5; ++i) {
        rtc::shortSleep(30, sys::STOP1);
        led.toggle();
    }
    fastClock();

    auto ms3 = rtc::getDate().todMillis();
    logf("3: %d", ms3);

#if STM32WL
    logf("isr %08x ssr %08x", +RTC[0x0C], +RTC[0x08]);
#endif

    auto ms = ms3 - ms1;
    assert(220 <= ms && ms <= 320);
}
