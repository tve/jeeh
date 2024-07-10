// Show a random file from an MRFS area.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

#include "mrfs.h"

int main () {
    initBoard();
    rng::init();

    mrfs::init((uint8_t*) 0x0800'0000 + fs.mapBase, fs.mapSize);

    char fnBuf [20];
    snprintf(fnBuf, sizeof fnBuf, "file-%d.txt", rng::rand() % 10);

    auto p = mrfs::open(fnBuf);

    if (p != nullptr) {
        auto a = (uint16_t) ((uint32_t) p - fs.mapBase) >> 5;
        logf("%04x: [%08x] %6d  20%d.%02d%02d  %s",
                a, p->check, p->size,
                p->time>>11, (p->time>>6)&0x1F, p->time&0x3F,
                p->name);
        logDump(p->begin(), p->end() - p->begin(), fnBuf);
    }

    while (true) { led.toggle(); sys::wait(250); }
}
