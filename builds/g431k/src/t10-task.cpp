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
    assert(doubler.tId == 1);

    Message m { doubler.tId, 'D', 111 };
    sys::call(m);
    assert(m.mLen == 222);

    assert(!m.inUse());
    sys::send(m);
    assert(m.mLen == 444); // no need to wait for reply

    auto& r = sys::recv();
    assert(&r == &m);
}
