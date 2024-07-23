// Event-driven EXTI example with external cap, using a safety timeout.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

struct Example {
    ExtIrq exti;
    Pin rc {"A12"}; // this pin has ≈800 nF capacitance tied to ground

    Message timer { '@', 'T' };
    Message rcPin { exti.dId, 'A', 12, (uint8_t*) ExtIrq::BOTH };

    Example () {
        timer.setCallback(this, &Example::onTimeout);
        rcPin.setCallback(this, &Example::onPinChange);
        start('I');
    }

    void start (char type) {
        assert(!timer.inUse());
        assert(!rcPin.inUse());
        led.toggle();

        sys::wait(100);
        logf("%c %d", type, cycles::millis());

        rc.mode(rc ? "D": "U"); // switch to pull-down or pull-up

        timer.mLen = 5000;
        //sys::send(timer); // generate a timeout in case pin change is missing

        sys::send(rcPin); // trigger on pin change, some 15..25 ms from now
    }

    void onPinChange (Message&) {
        //assert(timer.inUse());
        sys::drop(timer, '@');
        assert(!timer.inUse());

        rc = +rc;     // force output to same state as currently read
        rc.mode("P"); // ... then enable push-pull mode

        start('+');
    }

    void onTimeout (Message &) {
        assert(rcPin.inUse());
        sys::drop(rcPin, exti.dId);
        assert(!rcPin.inUse());

        start('T');
    }
};

int main () {
    initBoard();
    Pin::config("B4:V15"); // EVENTOUT

    Example activity;

    while (true) {
        Tracer<10> pt;
        sys::recv();
    }
}

uint8_t jeeh::lowestPower (uint8_t power, uint16_t) {
    Tracer<11> pt;
    return sys::STOP2;
}

void jeeh::resumePower () {
    Tracer<12> pt;
}
