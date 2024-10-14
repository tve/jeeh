#include <jee.h>
using namespace jeeh;
#define MUST_FAIL 1
#include "test.h"

int main () {
    Tester t;

    __builtin_trap();  // force a hard fault
}
