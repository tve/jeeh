#include <jee.h>
using namespace jeeh;
#include "defs.h"
#include "test.h"

int main () {
    Tester t;

    Pin led (LED);
    led.mode("P");

    assert(led == 0);
    led = 1;
    assert(led == 1);
}
