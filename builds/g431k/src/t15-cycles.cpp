#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

    cycles::init();

    auto c = cycles::count();
    sys::wait(10);
    c = cycles::count() - c;

    logf("wait: %d cycles, %d us", c, c / (SystemCoreClock / 1'000'000));

    c = cycles::count();
    itmWrite("by design, this message has exactly 50 characters\n", 50);
    c = cycles::count() - c;

    logf("itm:  %d cycles, %d us", c, c / (SystemCoreClock / 1'000'000));
}
