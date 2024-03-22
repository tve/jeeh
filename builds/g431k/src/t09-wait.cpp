#include <jee.h>
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

//extern void initTicker ();
//initTicker();

    itmWrite("10\n", 3);
    sys::wait(10);
    itmWrite("11\n", 3);
    assert(0);
}
