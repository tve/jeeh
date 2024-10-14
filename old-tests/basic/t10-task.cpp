#include <jee.h>
using namespace jeeh;
#include "test.h"

struct MyObj {
    int v = 1;
    Message m = { '@', 'T', 5 };

    void ping (Message&) {
        logf("ping %d", v++);

        m.setCallback(this, &MyObj::pong);
        sys::send(m);
    }

    void pong (Message&) {
        logf("pong %d", v);
        v += 10;
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
    sys::recv();

    logf("13");
    assert(o.v == 12);

    logDump(&m, sizeof m);
}
