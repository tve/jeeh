// Format an MRFS area.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

#include "filesys.h"

int main () {
    initBoard();
    rtc::set(DateTime {}); // set date to the compilation time of this code

    cycles::clear();
    fs.format();
    logf("format %d ms", cycles::millis());

    while (true) { led.toggle(); sys::wait(250); }
}
