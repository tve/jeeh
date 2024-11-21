#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    while (true) {
        auto s = cycles::millis() / 1000;
        logf("%02d:%02d", s/60, s%60);
        cycles::msBusy(1000);
    }
}
