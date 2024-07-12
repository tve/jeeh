// Dump the contents of an SD card.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#include "sdio.h"
#include "spi-sdcard.h"

template< typename SDIO >
struct SdWrap {
    SDIO& sdio;

    SdWrap (SDIO& s) : sdio (s) {}

    int rwBlock(uint8_t rw, uint32_t page, uint8_t* buf) const {
        Message m { sdio.dId, 'B', 0, (uint8_t*) page };
logf("10 %d", page);
        while (true) {
            sys::call(m);
            if (m.mLen == 0)
                break;
logf("11 %d %d", page, m.mLen);
            sys::wait(1);
        }
        m.mTag = rw;
        m.mLen = 512;
        m.mPtr = buf;
logf("12");
        sys::call(m);
logf("13");
        return 512;
    }

    int readBlock (uint32_t page, uint8_t* buf) const {
        return rwBlock('R', page, buf);
    }

    int writeBlock (uint32_t page, uint8_t const* buf) const {
        return rwBlock('W', page, buf);
    }
};

void sdTest () {
    // F7508-DK:
    //  PC8  D0  MISO
    //  PC9  D1
    //  PC10 D2
    //  PC11 D3  NSEL
    //  PC12 CLK SCLK
    //  PC13 Detect
    //  PD2  CMD MOSI

    Sdio sdio;
    sdio.init();

    Message m { sdio.dId, 'I' };
    sys::call(m);
    logf("cap %d", m.mPtr);

    SdWrap sd (sdio);

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
