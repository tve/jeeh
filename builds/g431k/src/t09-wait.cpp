#include <jee.h>
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

    itmWrite("10\n", 3);
    sys::wait(10);
    itmWrite("11\n", 3);
    sys::wait(20);
    itmWrite("12\n", 3);
    sys::wait(30);
    itmWrite("13\n", 3);
}
