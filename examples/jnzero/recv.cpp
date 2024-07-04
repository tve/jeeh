// Dump received RFM69 power metering packets to the serial port.

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
            //logf("rssi %d lna %d afc %d @ %02d:%02d:%02d",
            //        rf.rssi, rf.lna, rf.afc, dt.hh, dt.mm, dt.ss);
            //logDump(rxBuf, n);

            int vals [12];
            auto i = 2, j = 0;
            while (i < n && j < 12) {
                auto v = 0;
                do
                    v = (v<<7) ^ rxBuf[i];
                while (rxBuf[i++] < 0x80);
                v ^= 0x80;
                vals[j++] = v;
                //logf(" %d", v);
            }

            if (vals[0] == 1 && j == 11) // it's a packet from p1mon.cpp
                // 11 values: 4x Wh, tariff, 2x 10W, gas, liter, valve
                // kwh: consume: normal low, produce: normal low
                logf("%02d:%02d:%02d    %d %d %d %d %d %d %d %d %d %d",
                        dt.hh, dt.mm, dt.ss,
                        vals[1], vals[2], vals[3], vals[4], vals[5],
                        vals[6], vals[7], vals[8], vals[9], vals[10]);
        }
        sys::wait(1000);
    }
}
