#include <jee.h>
using namespace jeeh;
#include "test.h"

struct Two : Task {
    void process (Message& msg) override {
        msg.mTag = 'b';
        sys::send(msg); // send reply w/ lowercase tag
    }
};

struct One : Task {
    void process (Message& msg) override {
        logf("20 %d %c", msg.mDst, msg.mTag);
        assert(msg.mDst == 0 && msg.mTag == 'A');

        Two two;
        Message m { two.mTag, 'B' };
        sys::call(m);

        msg.mTag = 'a';
        sys::send(msg); // send reply w/ lowercase tag
    }
};

int main () {
    Tester t;

    One one;
    Message m { one.mTag, 'A' };
    sys::call(m);
    assert(m.mTag == 'a');
}
