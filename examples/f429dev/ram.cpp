// Access PSRAM and SDRAM memory on the HAOYU STM32F429GI board.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "memtest.h"

int main () {
    initBoard();
    printf("%s: ram @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);

    initFmcPins();
    initPsRam();
    initSdRam();

    auto SRAM = (uint8_t*) 0x6C00'0000;
    for (auto i = 0; i < 32; ++i)
        SRAM[i] = i+0x40;
    dumpHex(SRAM, 32, "4 MB PSRAM");
    printf(" %d errors\n", memTests((uint32_t) SRAM, 4<<20));

    ledR.toggle();

    auto SDRAM = (uint8_t*) 0xC000'0000;
    for (auto i = 0; i < 32; ++i)
        SDRAM[i] = i+0x60;
    dumpHex(SDRAM, 32, "32 MB SDRAM");
    printf(" %d errors\n", memTests((uint32_t) SDRAM, 32<<20));

    while (true) {
        ledL.toggle();
        sys::wait(500);
    }
}
