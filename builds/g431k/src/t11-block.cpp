#include <jee.h>
using namespace jeeh;
#include "test.h"

struct Doubler : Task {
    int process (Message& msg) override {
        msg.mLen *= 2;
        sys::wait(10); // nested blocking call
        sys::send(msg);
        return 0;
    }
};

int main () {
    Tester t;

    Doubler doubler;
    assert(doubler.tId == 1);

    Message m { doubler.tId, 'D', 222 };
    sys::call(m);
    assert(m.mLen == 444);

    assert(!m.inUse());
    sys::call(m);
    assert(m.mLen == 888);
}
