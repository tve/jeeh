#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "test.h"

constexpr Pin led (LED);

ExtIrq extirq;

int exti (Message&) {
    constexpr Pin in (EXT_IN);
    in.mode("F");

    Message msg {extirq.dId, EXT_IN[0], in.pin(), (uint8_t*) ExtIrq::BOTH};

    while (true) {
        sys::call(msg); // configure and block until pin changes
        led.toggle();
    }
}

int main () {
    Tester t;

    uint32_t stack [300];
    sys::init(stack);

    constexpr Pin out (EXT_OUT);
    out.mode("P");

    uint32_t extiStack [150];
    sys::fork(extiStack, exti);

    for (int i = 0; i < 5; ++i) {
        out = 1;
        sys::wait(100);
        out = 0;
        sys::wait(100);
    }

    fail();
}
