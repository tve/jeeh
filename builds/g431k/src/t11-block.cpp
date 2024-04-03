#include <jee.h>
using namespace jeeh;
#include "test.h"

struct Doubler : Task {
    int process (Message& msg) override {
        int x;
logf("10 %p %d %d %p", &msg, msg.mDst, msg.mLen, &x);
        //sys::wait(5); // nested blocking call
logf("11");
        msg.mLen *= 2;
        sys::send(msg);
logf("12");
        return 0;
    }
};

int main () {
    Tester t;

    Doubler doubler;
    assert(doubler.mTag == 1);

    Message m { doubler.mTag, 'D', 222 };
logf("20 %p %d %d", &m, m.mDst, m.mLen);
    sys::call(m);
    assert(m.mLen == 444);

logf("21");
    sys::wait(10);

logf("22");
    assert(!m.inUse());
    sys::call(m);
    assert(m.mLen == 888);
logf("23");
}
