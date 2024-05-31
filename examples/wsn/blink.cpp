// Minimal LED blinker, using a busy loop as delay.

#include <jee.h>
using namespace jeeh;
#include "defs.h"

int main () {
    rtc::init();

    Pin led (LED);  // defined in platformio.ini
    led.mode("P");  // push-pull output

    while (true) {
        led.toggle();
        //rtc::shortSleep(100, sys::STOP2);
        sys::wait(250);
    }
}
