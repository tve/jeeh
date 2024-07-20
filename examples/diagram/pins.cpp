// Toggle each pin in sequence to verify the Logic Analyser connection.

#include <jee.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    int i = 0;
    while (true) {
        tracePins[i] = 0;
        i = (i+1) % sizeof tracePins;
        tracePins[i] = 1;
    }
}
