// Minimal LED blinker.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    DateTime now; // the time this code was compiled, that is ...
    if (rtc::getSecs() < 367 * 86400) // 2001/01/02
        rtc::set(now); // only set if the date was clearly not inited

    while (true) {
        auto dt = rtc::getDate();
        auto ms = dt.todMillis();
        logf("20%02d/%02d/%02d %02d:%02d:%02d.%03d",
                dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, ms%1000);

        led = 0; // inverted logic
        sys::wait(100);
        led = 1;
        sys::wait(400);
    }
}
