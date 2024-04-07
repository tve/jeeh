#include <jee.h>
using namespace jeeh;
#include "test.h"

struct Doubler : Task {
    int process (Message& msg) override {
        msg.mLen *= 2;
        sys::send(msg);
        return 0;
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
