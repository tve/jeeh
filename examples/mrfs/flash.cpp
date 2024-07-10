// MRFS on internal flash.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/flash.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    constexpr auto BASE = 16*1024; // don't erase this code itself
    constexpr auto NUMP = 3;
    constexpr auto N32B = 32;
    constexpr auto STEP = 40;

    auto psz = flash::pageSize(0);
    logf("page size %d", psz);
    assert(N32B * STEP < psz);

    for (auto i = 0; i < NUMP; ++i) {
        auto offset = BASE + i * psz;

        cycles::clear();
        flash::erase(offset);
        logf("erase %x: %d ms/%db", i, cycles::millis(), psz);
        offset += psz;
    }

    logDump((uint32_t*) (0x0800'0000 + BASE), 80, "empty");

    for (auto i = 0; i < NUMP; ++i) {
        auto offset = BASE + i * psz;

        uint32_t buf [8];
        for (auto j = 0U; j < sizeof buf; ++j)
            ((uint8_t*) buf)[j] = j + i;

        cycles::clear();
        for (auto i = 0; i < N32B; ++i) {
            flash::write8w(offset, buf);
            offset += STEP;
        }
        logf("write %x: %d ms/%db", i, cycles::millis(), N32B * 32);
    }

    for (auto i = 0; i < NUMP; ++i) {
        auto offset = BASE + i * psz;

        logDump((uint32_t*) (0x0800'0000 + offset), 80, "verify");
    }

    while (true) { led.toggle(); sys::wait(250); }
}
