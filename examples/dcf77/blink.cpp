// Minimal LED blinker

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    while (true) {
        led = 1;
        cycles::msBusy(100);
        led = 0;
        cycles::msBusy(400);
    }
}
