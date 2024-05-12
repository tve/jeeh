#include <jee.h>
using namespace jeeh;
#include "test.h"

struct One : Task {
    void process (Message& msg) override {
        logf("20 %d %c", msg.mDst, msg.mTag);
        assert(msg.mTag == 'A');
        assert(msg.mDst == 0); // came from task 0 (main thread)

        Message m { 2, 'B' };
        sys::call(m);

        logf("21 %c", m.mTag);

        msg.mTag = 'a';
        sys::send(msg); // send reply w/ lowercase tag

        logf("22");
    }
};

struct Two : Task {
    void process (Message& msg) override {
        logf("30 %d %c", msg.mDst, msg.mTag);
        assert(msg.mTag == 'B');
        assert(msg.mDst == 1); // came from task 1

        msg.mTag = 'b';
        sys::send(msg); // send reply w/ lowercase tag

        logf("31");
    }
};

int main () {
    Tester t;

    One one; assert(one.mTag == 1);
    Two two; assert(two.mTag == 2);

    logf("10");

    Message m { 1, 'A' };
    sys::call(m);

    logf("11 %c", m.mTag);
    assert(m.mTag == 'a');
}
