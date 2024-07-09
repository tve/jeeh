// List an MRFS area.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

#include "filesys.h"
#include "mrfs.h"
#include "mrfs-arch.h"

int main () {
    initBoard();

    mrfs::init((uint8_t*) 0x0800'0000 + fs.mapBase, fs.mapSize);

    mrfs::File* p = nullptr;
    while (mrfs::readDir(p)) {
        auto a = (uint16_t) ((uint32_t) p - fs.mapBase) >> 5;
        logf("%04x: [%08x] %6d %d.%02d%02d %s",
                a, p->check, p->size,
                p->time>>11, (p->time>>6)&0x1F, p->time&0x3F,
                p->name);
    }

    while (true) { led.toggle(); sys::wait(250); }
}
