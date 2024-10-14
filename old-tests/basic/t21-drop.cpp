#include <jee.h>
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

    Message m1 { '@', 'T', 5 };
    sys::send(m1);

    Message m2 { '@', 'T', 20 };
    sys::send(m2);

    assert(m1.inUse());
    assert(m2.inUse());

    sys::wait(10);
    // m1 has now expired and is pending in this thread's input queue
    // m2 has not yet expired and is still in the ticker chain

    assert(m1.inUse());
    assert(m2.inUse());

    auto& m = sys::recv();
    assert(&m == &m1);
    assert(!m.inUse());

    auto f1 = sys::drop(m1, '@');
    assert(!f1); // drop failed, already picked up
    assert(!m.inUse());

    auto f2 = sys::drop(m2, '@');
    assert(f2); // drop succeeded
    assert(!m2.inUse());

    auto f3 = sys::drop(m1, '@');
    assert(!f3); // can't drop when not in use
}
