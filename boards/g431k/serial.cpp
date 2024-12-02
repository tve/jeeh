#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    auto s = " ABCDEFGHIJKLMNOPQRSTUVWXYZ 1234567890"
             " abcdefghijklmnopqrstuvwxyz 1234567890 /\n";
    auto n = strlen(s);

    // send over 3000 chars @ 10 Mbd, then pause, then repeat
    while (true) {
        uint32_t t = cycles::micros(), c = 0;
        for (auto i = 0U; i < n-2; ++i) {
            console.write(s + i, n - i);
            c += n - i;
        }
        t = cycles::micros() - t;

        led = 1;
        cycles::msBusy(100);

        logf("%d ch, %d µs", c, t); // polled, but uart is idle by now

        led = 0;
        cycles::msBusy(400);
    }
}
