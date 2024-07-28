// Toggle each pin in sequence to verify the Logic Analyser connection.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    auto i = sizeof tracePins - 1;
    while (true) {
        tracePins[i] = 0;
        i = (i+1) % sizeof tracePins;
        tracePins[i] = 1;
    }
}
