// LED blinker, with RTC seconds sent to the UART console.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard("blink"); // in defs.h

    while (true) {
        printf("%d\n", rtc::getSecs()); // F1 can't read sub-seconds from RTC
        led.toggle();
        sys::wait(500);
    }
}
