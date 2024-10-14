// Use sys::coma to go into long-term sleep modes.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    slowClock(false);

    rtc::init();
    sys::coma(2, sys::SHUTDOWN);

    fail(); // blink LED
}
