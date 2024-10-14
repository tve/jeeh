// Format an MRFS area.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    uint8_t buf [0x2000];
    for (auto i = 0U; i < fs.size; i += sizeof buf) {
        fs.read(i, buf, sizeof buf);
        logDump(buf, sizeof buf);
    }

    while (true) { led.toggle(); sys::wait(250); }
}
