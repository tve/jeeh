// Use the watchdog to reset the system after a while.

#include <jee.h>
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

    auto c = dog::resetCause();
    logf("cause %d", c);

    switch (c) {
        case 2: // power-up, this happens on first-time use
        case 1: // reset
            dog::init();
            logf("10");
            sys::wait(300);
            logf("11");
            sys::wait(300);
            logf("12");
            dog::reload(1);
            logf("13");
            sys::wait(5);
            fail();

        case -1: // watchdog
            dog::init(1);
            for (auto i = 0; i < 5; ++i) {
                sys::wait(10);
                dog::kick();
            }
            logf("14");
#if !STM32F1 // TODO
            dog::reload(40);
#endif
            sys::wait(3);
            logf("15");
            break;

        default:
            fail();
    }
}
