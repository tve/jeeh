// LED blinker, with RTC seconds sent to the UART console.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

//#define RF69_SPI_BULK 1
#include "spi-rf69-v1.h"

SpiGpio spi;
RF69 rf (spi);

constexpr Pin nrst ("A8");
constexpr Pin dio0 ("B5");
constexpr Pin dio1 ("B3");
constexpr Pin dio2 ("B4");
constexpr Pin dio3 ("B0");
constexpr Pin dio5 ("A15");

int main () {
    initBoard("poll"); // in defs.h

    AFIO[0x04](24,3) = 2;

    nrst.mode("P");
    dio0.mode("D");
    dio1.mode("D");
    dio2.mode("D");
    dio3.mode("D");
    dio5.mode("D");

    nrst = 1;
    sys::wait(10);
    nrst = 0;
    sys::wait(10);

    spi.init("A7,A6,A5,A4"); // div=0 @ 16 MHz: 8 Mhz
    rf.init(63, 42, 8686);  // node 63, group 42, 868.3 MHz
    rf.txPower(0);

    while (true) {
        uint8_t buf [60];
        auto n = rf.receive(buf, sizeof buf);
        if (n > 0) {
            printf("rssi %d lna %d afc %d @ %d\n",
                    rf.rssi, rf.lna, rf.afc, rtc::getSecs());
            logDump(buf, n);
        }
        sys::wait(100);
    }
}
