#include <jee.h>
using namespace jeeh;
#include "defs.h"
#include "test.h"

int main () {
    Tester t;

    logf("hello");
    auto p = sys::pool(1);
    auto q = sys::pool(1);
    logf("%p %p", p, q);
    assert(q == p + 8);
}
