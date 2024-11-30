// Minimal LED blinker, using a busy loop as delay.

#include <jee.h>
using namespace jeeh;
#include "defs.h"

void delayLoop (int n) {
    n *= SystemCoreClock / 5000;
    for (int i = 0; i < n; ++i)
        asm ("");
}

int main () {
    while (true) {
        led = 1;
        delayLoop(100);
        led = 0;
        delayLoop(400);
    }
}
