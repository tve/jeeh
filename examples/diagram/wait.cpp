// Examine the control flow of sys::wait.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    while (true) {
        Tracer<10> pt;
        sys::wait(2);
    }
}
