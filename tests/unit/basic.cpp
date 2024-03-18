#include <jee.h>
using namespace jeeh;
#include "defs.h"

namespace jeeh {
    void fail (char const*, int) {
        itmWrite("FAIL\n", 5);
        while (true) {}
    }
} // namespace jeeh

    int main () {
        Pin led (LED);
        led.mode("P");

        assert(led == 0);
        led = 1;
        assert(led == 1);

        itmWrite("12345678912345678912345678912345678900001234567890\n", 51);
        itmWrite("OK\n", 3);
    }
