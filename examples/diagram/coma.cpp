// Examine the control flow of sys::coma.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    while (true) {
        Tracer<10> pt;
        sys::wait(100);
    }
}
