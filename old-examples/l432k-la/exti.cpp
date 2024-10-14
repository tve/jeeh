// Event-driven EXTI example with external cap, using a safety timeout.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

struct Example {
    ExtIrq exti;
    Pin rc {"A12"}; // this pin has ≈800 nF capacitance tied to ground
    Message delay {}, timer {}, rcPin {};

    Example () {
        delay.setCallback(this, &Example::onDelay);
        timer.setCallback(this, &Example::onTimeout);
        rcPin.setCallback(this, &Example::onPinChange);
        start('i');
    }

    void start (char type) {
        assert(!delay.inUse());
        assert(!timer.inUse());
        assert(!rcPin.inUse());
        //led.toggle(); // oops, this interferes with Tracer<11>

        logf("%c %04d", type, rtc::getDate().todMillis() % 10'000);

        logf("z %d", rtc::shortSleep(50, sys::STOP2));

        delay.mDst = '@';
        delay.mTag = 'T';
        delay.mLen = 100;
        sys::send(delay);
    }

    void onDelay (Message&) {
        logf("D");
        rc.mode(rc ? "D": "U"); // switch to pull-down or pull-up

        timer.mDst = '@';
        timer.mTag = 'T';
        timer.mLen = 55;
        sys::send(timer); // generate a timeout in case pin change is missing

        rcPin.mDst = exti.dId;
        rcPin.mTag = 'A';
        rcPin.mLen = 12;
        rcPin.mPtr = (uint8_t*) ExtIrq::BOTH;
        sys::send(rcPin); // trigger on pin change, some 15..25 ms from now
    }

    void onPinChange (Message&) {
        logf("P");
        assert(timer.inUse());
        sys::drop(timer, '@');
        assert(!timer.inUse());

        rc = +rc;     // force output to same state as currently read
        rc.mode("P"); // ... then enable push-pull mode

        start('p');
    }

    void onTimeout (Message &) {
        logf("T");
        assert(rcPin.inUse());
        sys::drop(rcPin, exti.dId);
        assert(!rcPin.inUse());

        start('t');
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
