#include <jee.h>
using namespace jeeh;
#include "test.h"

// FIXME no longer useful without tasks

struct Two {
    void process (Message&) {
        sys::wait(20);

        //msg.mTag = 'b';
        //reply(msg);

        sys::wait(21);
    }
};

struct One {
    void process (Message&) {
        sys::wait(10);

        //Two two;

        //Message m { two.id(), 'B' };
        //sys::call(m);
        //assert(m.mTag == 'b');

        sys::wait(11);

        //msg.mTag = 'a';
        //reply(msg);

        sys::wait(12);
    }
};

int main () {
    Tester t;

    //One one;

    //Message m { one.id(), 'A' };
    //sys::call(m);
    //assert(m.mTag == 'a');
}
