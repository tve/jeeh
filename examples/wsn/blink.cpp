// Minimal LED blinker, using a busy loop as delay.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard("blink");

    while (true) {
        logf("%d", rtc::getSecs());
        led.toggle();
        //rtc::shortSleep(500, sys::STOP2);
        sys::wait(500);
    }
}
