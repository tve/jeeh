// Test external PSRAM memory.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#include <jee/util/memtest.h>

int main () {
    initBoard(); // in defs.h
    initFsmcPins();

    constexpr uint32_t addr = 0x6000'0000, size = 512; // kB
    logf("%d kB @ 0x%08x", size, addr);
    initPsram();

    while (true) {
        led.toggle();
        logf(" %d errors", util::memTests(addr, size * 1024));
    }
}
