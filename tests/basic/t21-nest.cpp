#include <jee.h>
using namespace jeeh;
#include "test.h"

struct Two : Task {
    void process (Message& msg) override {
        msg.mTag = 'b';
        reply(msg);
    }
};

struct One : Task {
    void process (Message& msg) override {
        Two two;

        Message m { two.id(), 'B' };
        sys::call(m);

        msg.mTag = 'a';
        reply(msg);
    }
};

int main () {
    Tester t;

    One one;

    Message m { one.id(), 'A' };
    sys::call(m);
    assert(m.mTag == 'a');
}
