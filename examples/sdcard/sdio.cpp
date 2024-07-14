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
        assert((uint32_t) buf % 4 == 0);
        Message m { sdio.dId, 'B', 0, (uint8_t*) page };
        while (true) {
            sys::call(m);
            if (m.mLen == 0)
                break;
logf("act");
            sys::wait(10);
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
        return rwBlock('W', page, (uint8_t*) buf);
    }
};

int main () {
    initBoard();

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

    auto mhz = SystemCoreClock/1'000'000;
    uint8_t buf [512] alignas (4);

    for (auto i = 0; i < 500; ++i) {
        auto t = cycles::count();
        sd.readBlock(fs.base + i, buf);
        t = cycles::count() - t;
        if (buf[0] != 0) {
            logf("read %d: %d us", i, t/mhz);
            logf("%d", i);
            logDump(buf, 64);
        }
    }

    auto show = [&](auto fn) {
        FileMap file (fs);
        auto n = file.open(fn);
        logf("%s: %d b", fn, n);
        for (auto i = 0; i < file.NFRAG; ++i)
            if (file.size[i] > 0)
                logf("  %d: %4d #%d", i, file.map[i], file.size[i]);
    };

    show("firmware.elf");
    show("f");
    show("g");
    show("h");
    show("x");
    show("list.cpp");

    FileMap file (fs);
    auto n = file.open("list.cpp");
    for (auto i = 0; i < n; i += 512) {
        logf("data @ %d", i);
        auto n = file.readBlock(i>>9, buf);
        assert(n == 512);
        logDump(buf, sizeof buf);
    }

    memset(buf, 0, sizeof buf);
    sd.readBlock(3000, buf);
    logDump(buf, 16, "3000: ??");
    memset(buf, 0, sizeof buf);
    sd.readBlock(3001, buf);
    logDump(buf, 16, "3001: ??");

sys::wait(100);
    memset(buf, 0x11, sizeof buf);
    sd.writeBlock(3000, buf);
sys::wait(100);
    memset(buf, 0x22, sizeof buf);
    sd.writeBlock(3001, buf);
sys::wait(100);

    memset(buf, 0, sizeof buf);
    sd.readBlock(3000, buf);
    logDump(buf, 16, "3000: 11");
    memset(buf, 0, sizeof buf);
    sd.readBlock(3001, buf);
    logDump(buf, 16, "3001: 22");

#if 0
sys::wait(100);
    memset(buf, 0x33, sizeof buf);
    sd.writeBlock(3000, buf);
sys::wait(100);
    memset(buf, 0x44, sizeof buf);
    sd.writeBlock(3001, buf);
sys::wait(100);

    memset(buf, 0, sizeof buf);
    sd.readBlock(3000, buf);
    logDump(buf, 16, "3000: 33");
    memset(buf, 0, sizeof buf);
    sd.readBlock(3001, buf);
    logDump(buf, 16, "3001: 44");
#endif

    while (true) { led.toggle(); sys::wait(250); }
}
