// Access PSRAM and SDRAM memory on the F769I Disco board.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "memtest.h"

auto ram = (uint8_t* const) 0xC000'0000;

int main () {
    initBoard();

    auto sdRam = initSdRam();

    for (auto i = 0; i < 32; ++i)
        sdRam[i] = i+0x60;
    logDump(sdRam, 32, "16 MB SDRAM");
    logf(" %d errors", memTests((uint32_t) sdRam, 16<<20));

    while (true) {
        led.toggle();
        sys::wait(500);
    }
}
