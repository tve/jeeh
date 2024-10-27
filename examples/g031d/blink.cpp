#include <jee.h>
using namespace jeeh;
#include "defs.h"

static void delayLoop (int n) {
    for (int i = 0; i < n * 3000; ++i)
        asm ("");
}

int main () {
    initBoard();

    while (true) {
        led.toggle();
        delayLoop(500);
    }
}
