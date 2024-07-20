// Examine the control flow when there is a background thread.

#include <jee.h>
using namespace jeeh;
#include "defs.h"

int blinker (Message& m) {
    while (true) {
        //Tracer<10> tr;
        led.toggle();
        sys::wait(3);
        (void) m; //sys::send(m);
        sys::wait(100);
    }
}

int main () {
    initBoard();

    while (true) {
        led.toggle();
        sys::wait(10);
        //for (auto i = 0; i < 2'000'000; ++i)
            //asm ("");  // prevents getting optimised away
    }
Message m {};
blinker(m);
    uint32_t stack [250];
    sys::init(stack);

    uint32_t blinkerStack [250];
    sys::fork(blinkerStack, blinker);

    while (true) {
        //Tracer<11> tr;
        sys::recv();
    }
}
