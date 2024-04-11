#include <jee.h>
using namespace jeeh;
#include "test.h"

struct One : Task {
    Message timer { '@', 'T', 10 };

    void process (Message& msg) override {
        logf("20 %d %c", msg.mDst, msg.mTag);
        switch (msg.mTag) {
            case 'I':
                logf("21");
                sys::call(timer);
                logf("22");
                break;
            case 'T':
                logf("23");
                sys::send(msg);
                logf("24");
                break;
        }
    }
};

int main () {
    Tester t;

    One one;
    sys::wait(0); // start Ticker

    Message m { one.mTag, 'I' };
    sys::send(m);

    sys::recv();
}
