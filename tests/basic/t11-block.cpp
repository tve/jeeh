#include <jee.h>
using namespace jeeh;
#include "test.h"

struct Doubler : Task {
    int process (Message& msg) override {
        logf("20");
        sys::wait(20); // nested blocking call
        logf("21");
        msg.mLen *= 2;
        sys::send(msg);
        logf("22");
        return 0;
    }
};

int main () {
    Tester t;

    Doubler doubler;
    assert(doubler.mTag == 1);

    logf("10");
    Message m { doubler.mTag, 'D', 222 };
    sys::call(m);
    assert(m.mLen == 444);

    sys::wait(10);

    logf("11");
    assert(!m.inUse());
    sys::call(m);
    assert(m.mLen == 888);

    logf("12");
}
