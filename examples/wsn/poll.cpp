// Dump received RFM69 packets to the serial port.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#include "spi-sync.h"
SpiSync spi ({ SPI_NAME.ADDR, ena::SPI_NAME, SPI_FREQ, SPI_CONF });

#include <jee/spi-rf69.h>
RF69 rf (spi);

constexpr Pin nrst ("B1");

int main () {
    initBoard("poll"); // in defs.h

    nrst.mode("P");

    nrst = 1;
    sys::wait(10);
    nrst = 0;
    sys::wait(10);

    //spi.init(SPI_PINS);
    spi.init(SPI_PINS, 10);
    rf.init(63, 42, 8686);  // node 63, group 42, 868.6 MHz
    rf.txPower(0);

    while (true) {
        uint8_t buf [60];
        auto n = rf.receive(buf, sizeof buf);
        if (n > 0) {
            logf("rssi %d lna %d afc %d @ %d\n",
                    rf.rssi, rf.lna, rf.afc, rtc::getSecs());
            logDump(buf, n);
        }
        sys::wait(100);
    }
}
