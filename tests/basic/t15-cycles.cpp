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

    logf("wait: %d us", c / (SystemCoreClock / 1'000'000));

    c = cycles::count();
    swoWrite("by design, this message has exactly 50 characters\n", 50);
    c = cycles::count() - c;

    logf("itm:  %d us", c / (SystemCoreClock / 1'000'000));
}
