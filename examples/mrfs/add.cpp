// Add dummy files to an MRFS area.

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

    char buf [50];
    for (auto i = 0; i < 26; ++i)
        buf[i] = 'a' + i;
    buf[26] = ' ';

    auto r = 1 + rng::rand() % 5;
    for (auto i = 0U; i < r; ++i) {
        snprintf(buf + 27, sizeof buf - 27, "%08X.", rng::rand());

        char fnBuf [20];
        snprintf(fnBuf, sizeof fnBuf, "file-%d.txt", rng::rand() % 10);

        auto n = 1 + rng::rand() % 10;
        logf("add %d/%d: %s %dx %db \"...%s\"",
                i+1, r, fnBuf, n, strlen(buf), buf + 23);

        mrfs::create(fnBuf);
        for (auto j = 0U; j < n; ++j)
            mrfs::write(buf, strlen(buf));
        mrfs::close();
    }

    mrfs::File* p = nullptr;
    while (mrfs::readDir(p)) {
        auto a = (uint16_t) ((uint32_t) p - fs.mapBase) >> 5;
        logf("%04x: %6d  20%06d.%02d%02d  %s",
                a, p->size,
                p->time >> 11, (p->time>>6) & 0x1F, p->time & 0x3F,
                p->name);
    }

    while (true) { led.toggle(); sys::wait(250); }
}
