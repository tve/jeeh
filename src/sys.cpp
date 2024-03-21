#include "jee.h"
using namespace jeeh;
#include <cstdarg>
#include <cstdio>

void jeeh::logf (char const* fmt ...) {
    constexpr IoReg<0xE000'0000> ITM;
    enum { TER=0xE00, TCR=0xE80 };

    // check for enabled ITM flags before generating any printf output
    if (ITM[TCR](0) && ITM[TER](0)) {
        static char buf [80];

        va_list ap;
        va_start(ap, fmt);
        auto n = vsnprintf(buf, sizeof buf, fmt, ap);
        va_end(ap);

        if (n >= (int) sizeof buf)
            n = sizeof buf;
        else if (n == 0 || buf[n-1] != '\n')
            ++n;
        buf[n-1] = '\n';

        itmWrite(buf, n);
    }
}

namespace jeeh::sys {

    void send (Message&) {}
    void wait (uint16_t ms) {
        for (auto i = 0; i < 4000 * ms; ++i)
            asm ("");  // prevents getting optimised away
    }

    int currId () { return 0; }

} // namespace jeeh::sys
