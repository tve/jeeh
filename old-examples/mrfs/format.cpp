// Format an MRFS area.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
#include "mrfs.h"
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();
    rtc::set(DateTime {}); // set date to the compilation time of this code

    cycles::clear();
    fs.wipe();
    logf("format %d ms", cycles::millis());

    while (true) { led.toggle(); sys::wait(250); }
}
