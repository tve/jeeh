// List an MRFS area.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

#include "mrfs.h"

int main () {
    initBoard();
    rtc::set(DateTime {}); // set date to the compilation time of this code

    Mrfs mrfs (fs);

    for (auto p = mrfs.readDir(); p != nullptr; p = mrfs.readDir(p)) {
        char buf [12];
        if (p->check != 0)
            snprintf(buf, sizeof buf, "[%08x]", p->check);
        else
            buf[0] = 0;
        logf("%04x: %10s %6d  20%06d.%02d%02d  %s",
                p->magic, buf, p->size,
                p->time >> 11, (p->time>>6) & 0x1F, p->time & 0x3F,
                p->name);
    }

    while (true) { led.toggle(); sys::wait(250); }
}
