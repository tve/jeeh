#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

const Pin pin {"A1"};
                 
int main () {
    initBoard();

    pin.mode("U");
    assert(pin == 1);
    pin.mode("D");
    assert(pin == 0);
    pin.mode("P");
    assert(pin == 0); // off is the default setting
    pin = 1;
    assert(pin == 1);
    pin = 0;
    assert(pin == 0);

    logf("OK");

    while (true) { led.toggle(); cycles::msBusy(500); }
}
