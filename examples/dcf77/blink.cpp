// Minimal LED blinker

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    while (true) {
        led = 1;
        ledN = 1;
        sys::wait(100);
        led = 0;
        ledN = 0;
        sys::wait(400);
    }
}
