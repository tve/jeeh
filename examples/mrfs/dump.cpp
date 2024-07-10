// Format an MRFS area.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    logDump((uint8_t const*) 0x0800'0000 + fs.mapBase, fs.mapSize);

    while (true) { led.toggle(); sys::wait(250); }
}
