#include <jee.h>
using namespace jeeh;
#define OWN_LOWPOWER 1
#include "test.h"

void LowPower::start (Message&) {
    logf("S");
}

void LowPower::finish () {
    logf(" F");
}

int main () {
    Tester t;

    logf("10");
    sys::wait(10);
    logf("11");

    Message m { '@', 'T', 20 };
    sys::send(m);

    logf("12");
    sys::recv();

    constexpr int delays [] = { 20, 50, 100, 250 };
    for (auto ms : delays) {
        logf("%d", ms);
        sys::wait(ms);
    }
}
