#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#if POLLED
i2c::Poll<I2C1.ADDR> i2cBus (ena::I2C1, 50);
#else
i2c::Gpio i2cBus;
#endif

i2c::Dev fram { i2cBus, 0x50 };

Pin trigger {"B12","P"};

constexpr auto N = 5;

void read32 (uint16_t addr, uint8_t* ptr) {
    trigger = 0;
    fram.read16be(addr, ptr, N);
}

void write32 (uint16_t addr, uint8_t const* ptr) {
    trigger = 1;
    fram.write16be(addr, ptr, N);
}

int main () {
    initBoard();
    i2cBus.init("B7:OUH4,B6", 1000); // sda scl

    dog::init(2);  // will fire approx. every 2 secs

    auto seq = 0;
    while (true) {
        dog::kick();
        logf("#%d", ++seq);

        uint8_t buf [N];
        memset(buf, 0xEE, sizeof buf);
        for (auto i = 0; i < 2; ++i)
            write32(N*i, buf);
        cycles::usBusy(40);

        for (auto i = 0; i < 2; ++i) {
            memset(buf, 0x55, sizeof buf);
            read32(N*i, buf);
            //logDump(buf, 16);
            logf("%02x %02x ... %02x %02x", buf[0], buf[1], buf[N-2], buf[N-1]);
        }
        cycles::usBusy(40);

        for (auto i = 0; i < 2; ++i) {
            memset(buf, i+128, sizeof buf);
            write32(N*i, buf);
        }
        cycles::usBusy(40);

        for (auto i = 0; i < 2; ++i) {
            memset(buf, 0xAA, sizeof buf);
            read32(N*i, buf);
            //logDump(buf, 16);
            logf("%02x %02x ... %02x %02x", buf[0], buf[1], buf[N-2], buf[N-1]);
        }

        fram.read16be(N, buf, 1);
        fram.read16be(0, buf+1, 1);
        logf("%02x %02x", buf[0], buf[1]);

        cycles::msBusy(1000);
        led.toggle();
    }
}
