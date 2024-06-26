// Dump received RFM69 packets to the serial port.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/rf69.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard(); // in defs.h

#if STM32F1
    AFIO[0x04](24,3) = 2;
#endif

#if USE_GPIO
    SpiGpio spi;
#elif USE_POLL
    SpiPoll<SPI_NAME.ADDR> spi (ena::SPI_NAME, SPI_FREQ);
#elif USE_SYNC
    SpiSync<SPI_TYPE> spi (SPI_CONF);
#elif USE_CALL
    SpiCall<SPI_TYPE> spi (SPI_CONF);
#endif

    spi.init(SPI_PINS, 10'000);

    nrst = 1;
    sys::wait(10);
    nrst = 0;
    sys::wait(10);

#if 1
    RF69 rf (spi);
    rf.init(63, 42, 8686);  // node 63, group 42, 868.6 MHz
    rf.txPower(0);

    while (true) {
        uint8_t rxBuf [64];
        auto n = rf.receive(rxBuf, sizeof rxBuf);
        if (n > 0) {
            logf("rssi %d lna %d afc %d @ %d",
                    rf.rssi, rf.lna, rf.afc, rtc::getSecs());
            logDump(rxBuf, n);
            break;
        }
        sys::wait(1000);
    }
#endif

    uint8_t buf [202];
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
    for (auto i = 2U; i <= sizeof buf; i += 100) {
        auto t = clock();
        spi.enable();
        spi.transfer(false, buf, i);
        spi.disable();
        t = clock() - t;
        logf("%4d bytes: %6d %s, diff %6d", i, t, units, t - u);
        u = t;
    }

    logf("\n>>> done");
}
