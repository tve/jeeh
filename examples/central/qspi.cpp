// Test external PSRAM memory.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#include "qspi.h"

int main () {
    initBoard(); // in defs.h

    printf("%d kB @ 0x%08x\n", 1<<(qspi::fsize-10), qspi::addr);

    qspi::init();

    const auto qmem = qspi::addr;

    for (int n = 0; n < 12; ++n) {
        printf("%2d: ", n);
        logDump(qmem + n*64, 64);
    }

    static uint32_t buf [64] { 0x1122, 0x3344, 0x5566, 0x7788 };
    auto MHz = SystemCoreClock/1'000'000;

    logDump(qmem + 1024, 64);

    cycles::init();
    auto t = cycles::count();
    qspi::write(4*1024, buf, 64);
    printf("write 256 B = %d us\n", (cycles::count() - t) / MHz);

    logDump(qmem + 1024, 64);

    t = cycles::count();
    qspi::erase(4*1024);
    printf("erase 4 kB = %d us\n", (cycles::count() - t) / MHz);

    logDump(qmem + 1024, 64);

    t = cycles::count();
    for (auto i = 0; i < 40; ++i)
        duffs(buf, qmem, 64);
    printf("read 10 kB = %d us\n", (cycles::count() - t) / MHz);

    qspi::deinit();

    while (true) { led.toggle(); sys::wait(250); }
}
