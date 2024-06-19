// Dump received RFM69 packets to the serial port.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/rf69.h>
using namespace jeeh;
#include "defs.h"

template< typename SPI >
void radioTest (SPI& spi) {
    auto khz = 10'000;
    spi.init(SPI_PINS, khz);
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

#if STM32L0
    auto units = "micros";
    auto clock = []() {
        return ((STK[0x4]-STK[0x8]) * 80) / (SystemCoreClock/100'000);
    };

    auto ticksPerMs = SystemCoreClock / 1000;
    STK[0x4] = (100*ticksPerMs)/8-1; // reload value
    STK[0x8] = 0;                    // current
    STK[0x0] = 0b011;                // control, clk/8 mode

#else
    auto units = "cycles";
    auto clock = cycles::count;
#endif

    uint32_t u = 0;
    for (auto i = 2; i <= 62; i += 20) {
        auto t = clock();
        spi.transfer(spi.R1, nullptr, 0);
        spi.transfer(spi.R2, buf, i);
        t = clock() - t;
        logf("%4d bytes: %6d %s, diff %5d", i, t, units, t - u);
        u = t;
    }

    spi.deinit();
}

int main () {
    initBoard("poll"); // in defs.h

#if STM32F1
    AFIO[0x04](24,3) = 2;
#endif

    SpiCall<SPI_TYPE> spi2 (SPI_CONF);

    if (0) {
        logf("\n>>> SpiGpio: bit-banged");
        SpiGpio spi;
        radioTest(spi);
    }
    if (1) {
        logf("\n>>> SpiPoll: polled h/w regs");
        //auto& spi = (SpiPoll<SPI_NAME.ADDR>&) spi2;
        SpiPoll<SPI_NAME.ADDR> spi (ena::SPI_NAME, SPI_FREQ);
        radioTest(spi);
    }
    if (0) {
        logf("\n>>> SpiSync: sync wfe loop");
        auto& spi = (SpiSync<SPI_TYPE>&) spi2;
        radioTest(spi);
    }
    if (0) {
        logf("\n>>> SpiCall: async msg call");
        radioTest(spi2);
    }
    logf("\n>>> done");
}
