#include <jee.h>
using namespace jeeh;
#include "test.h"

struct Doubler : Task {
    void process (Message& msg) override {
        msg.mLen *= 2;
        sys::send(msg);
    }
};

int main () {
    Tester t;

    Doubler doubler;
    assert(doubler.mTag == 1);

    Message m { doubler.mTag, 'D', 111 };
    sys::call(m);
    assert(m.mLen == 222);

    sys::wait(10);

    assert(!m.inUse());
    sys::call(m);
    assert(m.mLen == 444);
}
