// Access PSRAM and SDRAM memory on the HAOYU STM32F429GI board.

#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/i2c.h>
#include <jee/spi.h>
#include <jee/uart.h>
using namespace jeeh;
#include "defs.h"
#include "memtest.h"

int main () {
    initBoard();

    initFmcPins();
    auto psRam = initPsRam();
    auto sdRam = initSdRam();

    for (auto i = 0; i < 32; ++i)
        psRam[i] = i+0x40;
    logDump(psRam, 32, "4 MB PSRAM");
    logf(" %d errors", memTests((uint32_t) psRam, 4<<20));

    ledR.toggle();

    for (auto i = 0; i < 32; ++i)
        sdRam[i] = i+0x60;
    logDump(sdRam, 32, "32 MB SDRAM");
    logf(" %d errors", memTests((uint32_t) sdRam, 32<<20));

    while (true) {
        ledL.toggle();
        cycles::msBusy(500);
    }
}
