// Random Number Generator test.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    rng::init();

    auto mhz = SystemCoreClock / 1'000'000;
    constexpr auto N = 1000;
    uint32_t buf [N];

    rng::Permutation<25> perm;

    while (true) {
        // random values
        for (auto n = 0; n < 5; ++n) {
            cycles::clear();
            for (auto i = 0; i < N; ++i)
                buf[i] = rng::rand();
            auto t = cycles::count();

            logf("%08x %08x %08x %08x - %d ns/word",
                    buf[0], buf[1], buf[2], buf[3], t/mhz, N);
        }

        // random pick
        for (auto n = 0; n < 5; ++n) {
            perm.init();

            char buf [80], *p = buf;
            int r;
            do {
                r = perm.next();
                p += snprintf(p, 4, " %d", r);
            } while (r >= 0);
            assert(p <= buf + sizeof buf);

            logf("%s", buf);
        }

        // random shuffle
        perm.init();
        for (auto n = 0; n < 5; ++n) {
            perm.shuffle();

            char buf [80], *p = buf;
            for (auto e : perm.choice)
                p += snprintf(p, 4, " %d", e);
            assert(p <= buf + sizeof buf);

            logf("%s", buf);
        }

        sys::wait(1000);
    }
}
