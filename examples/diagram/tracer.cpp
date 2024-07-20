// Use PinTracer objects to auto-set/-clear pins in a lexical scope.

#include <jee.h>
using namespace jeeh;
#include "defs.h"

template< int N >
struct PinTracer {
    PinTracer () { tracePins[N] = 1; }
    ~PinTracer () { tracePins[N] = 0; }
};

void one () {
    PinTracer<1> pt;
}

void two () {
    PinTracer<2> pt;
    one();
}

void three () {
    PinTracer<3> pt;
    two();
}

int main () {
    initBoard();

    while (true) {
        PinTracer<4> pt;
        three();
    }
}
