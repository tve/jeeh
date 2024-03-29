#include <jee.h>
using namespace jeeh;
#include "test.h"

int myThread (Message&) {
    logf("20");
    sys::wait(10);
    logf("21");
    sys::wait(20);
    logf("22");

    return 12345;
}

int main () {
    Tester t;

    uint32_t stack [300];
    sys::init(stack);

    logf("10");

    uint32_t myStack [200];
    auto& my = sys::fork(myStack, myThread);

    logf("11");
    sys::wait(5);
    logf("12");
    sys::wait(10);
    logf("13");
    sys::wait(20);
    logf("14");

    auto& r = sys::recv();
    assert(&r == &my);
    logf("15 %d", r.mArg);
    assert(r.mArg == 12345);
}
