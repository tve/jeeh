// SPI Flash test.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard("rand");

    rng::init();

    auto mhz = SystemCoreClock / 1'000'000;
    constexpr auto N = 1000;
    uint32_t buf [N];

    rng::Permutation<25> perm;

    while (true) {
        for (auto n = 0; n < 5; ++n) {
            cycles::clear();
            for (auto i = 0; i < N; ++i)
                buf[i] = rng::rand();
            auto t = cycles::count();

            logf("%08x %08x %08x %08x - %d ns/word",
                    buf[0], buf[1], buf[2], buf[3], t/mhz, N);
        }

        for (auto n = 0; n < 5; ++n) {
            perm.init();
            char buf [80], *p = buf;
            for (auto i = 0; i < 26; ++i)
                p += snprintf(p, 4, " %d", perm.next());
            assert(p < buf + sizeof buf);
            *p = 0;
            logf("%s", buf);
        }

        sys::wait(1000);
    }
}
