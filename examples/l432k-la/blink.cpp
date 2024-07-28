// Toggle the on-board LED.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    //initBoard();
    led.mode("P");

    while (true) {
        led.toggle();
        sys::wait(250);
    }
}
