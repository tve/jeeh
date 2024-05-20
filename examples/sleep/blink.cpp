// Blink the LED, with 5s shutdowns in between.

#include <jee.h>
#include "defs.h"
using namespace jeeh;

void LowPower::start (Message&) {}
void LowPower::finish () {}

int main () {
    for (auto i = 0; i < 100; ++i) asm ("");
    slowClock(false);           // 100 kHz
    for (auto i = 0; i < 100; ++i) asm ("");

    constexpr Pin led (LED);
    led.mode("P");

    rtc::init();
    for (auto i = 0; i < 50; ++i) asm ("");

    led = 1;
    rtc::deepSleep(10, 2);      // STOP2
    for (auto i = 0; i < 50; ++i) asm ("");

    led = 0;
    rtc::deepSleep(5000, 4);    // SHUTDOWN
    while (true) { led.toggle(); sys::wait(100); } // never reached
}
