#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

    cycles::init();
    auto c = cycles::count();

    sys::wait(5);

    c = cycles::count() - c;
    logf("%d cycles, %d us", c, c / (SystemCoreClock / 1'000'000));
}
