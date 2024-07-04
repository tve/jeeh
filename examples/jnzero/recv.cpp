// Dump received RFM69 packets to the serial port.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/rf69.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard(); // in defs.h

    spiBus.init(SPI_PINS, 10'000);

#if 0
    nrst = 1;
    sys::wait(10);
    nrst = 0;
    sys::wait(10);
#endif

    RF69 rf (spiBus);
    rf.init(63, 42, 8686);  // node 63, group 42, 868.6 MHz
    rf.txPower(0);

    while (true) {
        uint8_t rxBuf [64];
        auto n = rf.receive(rxBuf, sizeof rxBuf);
        if (n > 0) {
            auto dt = rtc::getDate();
            logf("rssi %d lna %d afc %d @ %02d:%02d:%02d",
                    rf.rssi, rf.lna, rf.afc, dt.hh, dt.mm, dt.ss);
            logDump(rxBuf, n);
        }
        sys::wait(1000);
    }
}
