// Test external PSRAM memory.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#include <jee/util/memtest.h>

int main () {
    initBoard("mem"); // in defs.h
    initFsmcPins();

    constexpr uint32_t addr = 0x6000'0000, size = 512; // kB
    printf("%d kB @ 0x%08x\n", size, addr);
    initPsram();

    while (true) {
        led.toggle();
        printf(" %d errors\n", util::memTests(addr, size * 1024));
    }
}
