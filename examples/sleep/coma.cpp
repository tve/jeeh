// Use sys::coma to go into long-term sleep modes.

#include <jee.h>
#include "defs.h"
using namespace jeeh;

int main () {
    slowClock(false);

    rtc::init();
    sys::coma(2, sys::SHUTDOWN);

    fail(); // blink LED
}
