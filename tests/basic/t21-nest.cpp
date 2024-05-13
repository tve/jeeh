#include <jee.h>
using namespace jeeh;
#include "test.h"

struct Two : Task {
    void process (Message& msg) override {
        sys::wait(20);

        msg.mTag = 'b';
        reply(msg);

        sys::wait(21);
    }
};

struct One : Task {
    void process (Message& msg) override {
        sys::wait(10);

        Two two;

        Message m { two.id(), 'B' };
        sys::call(m);
        assert(m.mTag == 'b');

        sys::wait(11);

        msg.mTag = 'a';
        reply(msg);

        sys::wait(12);
    }
};

int main () {
    Tester t;

    One one;

    Message m { one.id(), 'A' };
    sys::call(m);
    assert(m.mTag == 'a');
}
