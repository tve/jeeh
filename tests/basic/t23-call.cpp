#include <jee.h>
using namespace jeeh;
#include "test.h"

struct One : Task {
    Message* orig;
    Message timer { '@', 'T', 10 };
    Message child { 2, 'X' };
    uint8_t state =0;

    // <0> send <1> reply
    // <2> send <3> reply
    // <4> call <5> reply
    // <6> wait <7> reply

    void process (Message& msg) override {
        switch (state++) {
            case 0:
            case 2:
                assert(msg.mTag <= 'B');
                orig = &msg;
                sys::send(timer);
                break;
            case 4:
                assert(msg.mTag == 'C');
                sys::call(child); // nested task call, see main @13
                break;
            case 6:
                assert(msg.mTag == 'D');
                sys::wait(30); // nested driver call
                break;
            case 1:
            case 3:
            case 5:
            case 7:
                sys::send(*orig);
                break;
            default:
                fail();
        }
    }
};

struct Two : Task {
    Message timer { '@', 'T', 20 };
    Message* orig;
    uint8_t state =10;

    // <10> send T <11> reply

    void process (Message& msg) override {
        switch (state++) {
            case 10:
                orig = &msg;
                sys::send(timer);
                break;
            case 11:
                sys::send(*orig);
                break;
            default:
                fail();
        }
    }
};

int main () {
    Tester t;

    One one;
    Two two;
    assert(one.mTag == 1);
    assert(two.mTag == 2);

    sys::wait(0); // start Ticker
    logf("10");

    Message m { 1, 'A' };
    sys::send(m);
    logf("11 %c", m.mTag);

    auto& r = sys::recv();
    logf("12 %c", m.mTag);
    assert(&r == &m);

    m.mTag = 'B';
    sys::call(m);
    logf("13 %c", m.mTag);

    m.mTag = 'C';
    sys::call(m);
    logf("14 %c", m.mTag);

    m.mTag = 'D';
    sys::call(m);
    logf("15 %c", m.mTag);
}
