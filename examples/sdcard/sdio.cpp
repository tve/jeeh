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
        while (true) {
            sys::call(m);
            if (m.mLen == 0)
                break;
            sys::wait(1);
        }
        m.mTag = rw;
        m.mLen = 512;
        m.mPtr = buf;
        sys::call(m);
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
    FatFS fs (sd);
    fs.init();

    uint8_t buf [512];
    for (auto i = 0; i < 500; ++i) {
        auto t = cycles::count();
        sd.readBlock(fs.base + i, buf);
        t = cycles::count() - t;
        if (buf[0] != 0) {
            logf("read %d: %d us", i, t/168);
            logf("%d", i);
            logDump(buf, 64);
        }
    }

#if 0

    // 8M = 256 fat entries x 32K
    typedef FileMap< decltype(fs), 257 > DiskMap;
    DiskMap diskMap (fs);
    auto limit = diskMap.open("FIRMWAREELF");
    logf("limit %d", limit);
#endif
}

int main () {
    initBoard();

    sdTest();

    while (true) { led.toggle(); sys::wait(250); }
}
