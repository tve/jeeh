#include <jee.h>
using namespace jeeh;
#include "defs.h"

static void delayLoop (int n) {
    for (int i = 0; i < n * 1000; ++i)
        asm ("");
}

int main () {
    while (true) {
        led.toggle();
        delayLoop(500);
    }
}
