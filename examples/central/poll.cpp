// Polled access to the RFM69 radio, via bit-banged or hardware SPI.

#include <jee.h>
#include <jee/hal.h>
#include <jee/spi-rf69.h>
using namespace jeeh;
#include "defs.h"

#include "spi-sync.h"

SpiSync spi ({ SPI_NAME.ADDR, ena::SPI_NAME, SPI_FREQ, SPI_CONF });
RF69 rf (spi);

constexpr Pin nrst ("F11");
// not attached:
//constexpr Pin dio0 ("A4");
//constexpr Pin dio1 ("B0");
//constexpr Pin dio2 ("B11");
//constexpr Pin dio3 ("H4");
//constexpr Pin dio5 ("H5");

int main () {
    initBoard("poll"); // in defs.h

    nrst.mode("P");
    //dio0.mode("D");
    //dio1.mode("D");
    //dio2.mode("D");
    //dio3.mode("D");
    //dio5.mode("D");

    nrst = 1;
    sys::wait(10);
    nrst = 0;
    sys::wait(10);

    spi.init(SPI_PINS, 10);
    rf.init(63, 42, 8686);  // node 63, group 42, 868.6 MHz
    rf.txPower(0);

    while (true) {
        uint8_t buf [60];
        auto n = rf.receive(buf, sizeof buf);
        if (n > 0) {
            logf("rssi %d lna %d afc %d @ %d",
                    rf.rssi, rf.lna, rf.afc, rtc::getSecs());
            logDump(buf, n);
        }
        sys::wait(100);
    }
}
