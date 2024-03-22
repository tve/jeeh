#include <jee.h>
using namespace jeeh;
#include "test.h"

int main () {
    Tester t;

    [[maybe_unused]] auto p = sys::pool(1);
    [[maybe_unused]] auto q = sys::pool(1);
    assert(q == p + 8);
}
