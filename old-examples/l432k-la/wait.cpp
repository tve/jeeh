// Examine the control flow of sys::wait.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    RCC[0x08](24,4) = 6;    // LSI
    Pin::config("A8:0");    // MCO

    while (true) {
        Tracer<10> pt;
        sys::wait(2);
    }
}
