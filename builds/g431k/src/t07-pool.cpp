#include <jee.h>
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

    auto p = sys::pool(1);
    auto q = sys::pool(1);
    assert(q == p + 8);
}
