#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    while (true) {
        led = 0;
        cycles::msBusy(100);
        led = 1;
        cycles::msBusy(400);
    }
}
