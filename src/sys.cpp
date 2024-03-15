#include "jee.h"

namespace jeeh::sys {

    void send (Message&) {}
    void wait (uint16_t ms) {
        for (auto i = 0; i < 4000 * ms; ++i)
            asm ("");  // prevents getting optimised away
    }

    int currId () { return 0; }

} // namespace jeeh::sys
