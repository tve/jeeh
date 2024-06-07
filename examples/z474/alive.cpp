// Keep the watchdog alive with an ever-later periodic callback.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard("alive");

    logf("reset %d", dog::resetCause());

    dog::init(2); // will fire approx. every 2 secs

    struct Kicker {
        Message timer { '@', 'T' };

        Kicker () { trigger(timer); } // start the cycle

        void trigger (Message&) {
            logf("kick @ %d ms", rtc::getDate().todMillis() % 10'000);
            dog::kick();

            static int ms = 1000;
            timer.mLen = ms; // timeout
            ms += 500;

            sys::send(timer.setCallback(this, &Kicker::trigger));
        }
    };

    Kicker kicker;

    int i = 0;
    while (true) {
        logf("loop %d", ++i);
        sys::recv();
    }
}
