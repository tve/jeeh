#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

constexpr auto N = 5;

template< typename T >
void loopBack (char const* s, T& u) {
    uint8_t buf [] = "abcdefghijklmnopqrstuvwxyz";
    u.write(buf, N);
    auto n = u.read(buf, N);
    logDump(buf, n, s);
}

int main () {
    initBoard();

    loopBack("A", uartA);
    loopBack("B", uartB);
    loopBack("C", uartC);
    loopBack("D", uartD);

while (true) { led.toggle(); cycles::msBusy(500); }

    auto s = " ABCDEFGHIJKLMNOPQRSTUVWXYZ 1234567890"
             " abcdefghijklmnopqrstuvwxyz 1234567890 /\n";
    auto n = strlen(s);

    // send over 3000 chars @ 1 Mbd, then pause, then repeat
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
