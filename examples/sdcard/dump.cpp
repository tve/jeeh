// Dump the contents of an SD card.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#include "spi-sdcard.h"

void sdTest () {
    // F7508-DK:
    //  PC8  D0  MISO
    //  PC9  D1
    //  PC10 D2
    //  PC11 D3  NSEL
    //  PC12 CLK SCLK
    //  PC13 Detect
    //  PD2  CMD MOSI
    spiBus.init("D2,C8,C12,C11", 1000);

    SdCard sd (spiBus);
    spiBus.rate = 100;
    sd.init();
    spiBus.rate = 1;

    uint8_t buf [512];
    for (auto i = 0; i < 500; ++i) {
        sd.readBlock(2048 + i, buf);
        if (buf[0] != 0) {
            logf("%d", i);
            logDump(buf, 128);
        }
    }

    FatFS fatFs (sd);

    // 8M = 256 fat entries x 32K
    typedef FileMap< decltype(fatFs), 257 > DiskMap;
    DiskMap diskMap (fatFs);
    auto limit = diskMap.open("FIRMWAREELF");
    logf("limit %d", limit);
}

int main () {
    initBoard();

    sdTest();

    while (true) { led.toggle(); sys::wait(250); }
}
