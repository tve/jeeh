#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    while (true) {
        logf("%d ms", rtc::getDate().todMillis());

        led1 = 1;
        cycles::msBusy(100);
        led1 = 0;
        cycles::msBusy(900);
    }
}
