// Dump received RFM69 packets to the serial port.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#include <jee/spi-rf69.h>
RF69 rf (spi);

#if STM32F1
constexpr Pin nrst ("A8");
// not attached:
//constexpr Pin dio0 ("B5");
//constexpr Pin dio1 ("B3");
//constexpr Pin dio2 ("B4");
//constexpr Pin dio3 ("B0");
//constexpr Pin dio5 ("A15");
#else
constexpr Pin nrst ("F11");
// not attached:
//constexpr Pin dio0 ("A4");
//constexpr Pin dio1 ("B0");
//constexpr Pin dio2 ("B11");
//constexpr Pin dio3 ("H4");
//constexpr Pin dio5 ("H5");
#endif

int main () {
    initBoard("poll"); // in defs.h

#if STM32F1
    AFIO[0x04](24,3) = 2;
#endif

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
