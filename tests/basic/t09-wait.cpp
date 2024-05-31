#include <jee.h>
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

    swoWrite("10\n", 3);
    sys::wait(10);
    swoWrite("11\n", 3);
    sys::wait(20);
    swoWrite("12\n", 3);
    sys::wait(30);
    swoWrite("13\n", 3);
}
