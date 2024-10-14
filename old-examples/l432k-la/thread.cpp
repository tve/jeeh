// Examine the control flow when there is a background thread.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int blinker (Message& m) {
    while (true) {
        Tracer<10> tr;
        led.toggle();
        sys::call(m);
        sys::wait(100);
    }
}

int main () {
    initBoard();

    uint32_t stack [250];
    sys::init(stack);

    uint32_t blinkerStack [250];
    sys::fork(blinkerStack, blinker);

    while (true) {
        Tracer<11> tr;
        auto& m = sys::recv();
        sys::send(m);
    }
}
