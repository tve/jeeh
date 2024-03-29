#include <jee.h>
using namespace jeeh;
#include "test.h"

int myThread (Message&) {
    logf("20");
    sys::wait(10);
    logf("21");

    return 12345;
}

int main () {
    Tester t;

    uint32_t stack [300];
    sys::init(stack);

    logf("10");

    uint32_t myStack [200];
    [[maybe_unused]] auto& my = sys::fork(myStack, myThread);

    logf("11");
    sys::wait(5);
    logf("12");
    sys::wait(10);
    logf("13");

    auto& r = sys::recv();
    assert(&r == &my);
    logf("14 %d", r.mArg);
    assert(r.mArg == 12345);
}
