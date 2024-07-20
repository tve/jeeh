// Toggle the on-board LED.

#include <jee.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    while (true) {
        led.toggle();

        for (auto i = 0; i < 2'000'000; ++i)
            asm ("");  // prevents getting optimised away
    }
}
