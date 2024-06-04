#include <jee.h>
using namespace jeeh;
#include "test.h"

struct MyObj {
    char const* s = "ping";

    void ping (Message&) {
        logf(s);
    }
};

int main () {
    Tester t;

    MyObj o;

    Message m { '@', 'T', 10 };
    m.setCallback(&o, &MyObj::ping);
    sys::send(m);

    logf("10");
    sys::wait(20);
    logf("11");
    sys::recv();
    logf("12");

    logDump(&m, sizeof m);
}
