// Dump received RFM69 packets to the serial port.

#include <jee.h>
#include <jee/hal.h>
#include <jee/spi-rf69.h>
using namespace jeeh;
#include "defs.h"

template< typename SPI >
void radioTest (SPI& spi) {
    RF69 rf (spi);

    nrst = 1;
    sys::wait(10);
    nrst = 0;
    sys::wait(10);

    rf.init(63, 42, 8686);  // node 63, group 42, 868.6 MHz
    rf.txPower(0);

    while (true) {
        uint8_t buf [60];
        auto n = rf.receive(buf, sizeof buf);
        if (n > 0) {
            logf("rssi %d lna %d afc %d @ %d",
                    rf.rssi, rf.lna, rf.afc, rtc::getSecs());
            logDump(buf, n);
            break;
        }
        sys::wait(100);
    }

    uint8_t buf [62];
    cycles::init();

    uint32_t u = 0;
    for (auto i = 2; i <= 62; i += 20) {
        auto t = cycles::count();
        spi.transfer(buf, buf, i);
        t = cycles::count() - t;
        logf("%4d bytes: %6d cycles, diff %5d", i, t, t - u);
        u = t;
    }
}

struct SpiSync : SpiDev {
    using SpiDev::SpiDev;
    using SpiDev::transfer;

    void transfer (uint8_t const* out, uint8_t* in, int len) const {
        SpiDev::Request req (out, in, len);
        SpiDev::transfer(req); // sync with wfe & sleep
        //logf("got sync");
    }
};

struct SpiAsync : SpiSync {
    using SpiSync::SpiSync;
    using SpiSync::transfer;

    void transfer (uint8_t const* out, uint8_t* in, int len) const {
        SpiDev::Request req (out, in, len);
        req.mDst = 'S'; // TODO yuck
        sys::call(req); // async with thread suspend
        //logf("got async");
    }
};

int main () {
    initBoard("poll"); // in defs.h

#if STM32F1
    AFIO[0x04](24,3) = 2;
#endif

    SpiAsync spi2 ({ SPI_NAME.ADDR, ena::SPI_NAME, SPI_FREQ, SPI_CONF });

    if (1) {   
        logf("\n>>> SpiGpio: bit-banged");
        SpiGpio spi;
        spi.init(SPI_PINS);
        radioTest(spi);
        spi.deinit();
    }
    if (1) {   
        logf("\n>>> SpiDev: polled h/w regs");
        auto& spi = (SpiHw&) spi2;
        spi.init(SPI_PINS, 10);
        radioTest(spi);
        spi.deinit();
    }
    if (1) {   
        logf("\n>>> SpiSync: sync wfe-loop");
        auto& spi = (SpiSync&) spi2;
        spi.init(SPI_PINS, 10);
        radioTest(spi);
        spi.deinit();
    }
    if (1) {   
        logf("\n>>> SpiAsync: async device");
        auto& spi = (SpiAsync&) spi2;
        spi.init(SPI_PINS, 10);
        radioTest(spi);
        spi.deinit();
    }
    logf("\n>>> done");
}
