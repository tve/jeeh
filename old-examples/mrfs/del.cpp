// Delete a file from an MRFS area.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

#include "mrfs.h"

int main () {
    initBoard();
    rng::init();

    Mrfs mrfs (fs);

    char fnBuf [20];
    snprintf(fnBuf, sizeof fnBuf, "file-%d.txt", rng::rand() % 10);

    mrfs.remove(fnBuf);

    for (auto p = mrfs.readDir(); p != nullptr; p = mrfs.readDir(p))
        logf("%04x: %6d  20%06d.%02d%02d  %s",
                p->magic, p->size,
                p->time>>11, (p->time>>6)&0x1F, p->time&0x3F,
                p->name);

    while (true) { led.toggle(); sys::wait(250); }
}
