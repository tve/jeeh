#include <jee.h>
#include "defs.h"
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

    Pin led (LED);
    led.mode("P");

#define W(s) logWriter(s "\n", sizeof s); led.toggle();
#include "pattern.h"

    led = 0;
}
