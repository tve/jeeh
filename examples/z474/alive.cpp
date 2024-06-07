// Keep the watchdog alive with an ever-later periodic callback.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    logf("reset %d", dog::resetCause());

    dog::init(2); // will fire approx. every 2 secs

    struct Kicker : Message {
        Kicker () : Message { '@', 'T' } {
            setCallback(this, &Kicker::trigger);
            trigger(*this); // start the cycle
        }

        void trigger (Message&) {
            logf("kick @ %d ms", rtc::getDate().todMillis() % 10'000);
            dog::kick();

            static int ms = 1000;
            mLen = ms; // timeout
            ms += 500;

            sys::send(*this);
        }
    };

    Kicker kicker;

    int i = 0;
    while (true) {
        logf("loop %d", ++i);
        sys::recv();
    }
}
