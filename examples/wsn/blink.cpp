// Minimal LED blinker, using a busy loop as delay.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    while (true) {
        logf("%d", rtc::getSecs());
        led.toggle();
        //rtc::shortSleep(100, sys::STOP2);
        sys::wait(250);
    }
}
