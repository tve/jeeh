#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

namespace ni2c {
    namespace i2c {
        struct Config {
            uint32_t addr =0;
            uint16_t ena =0;
            uint8_t mhz =0;
            Irq evIrq ={}, erIrq ={}, txIrq ={}, rxIrq ={};
            uint32_t dmaAddr =0;
            uint8_t dmaTx =0, dmaRx =0;
        };

        using Gpio = ::jeeh::i2c::Gpio;

        template< typename DEV, Config const& CFG >
        struct Dev {
            static constexpr IoReg<CFG.addr> I2C {};
            DmaConfig<CFG.dmaAddr,CFG.dmaTx,CFG.dmaRx> dma;

            uint8_t sub;

            Dev (uint8_t s) : sub (s) {
            }

            void read (uint8_t reg) const {
                logf("sub %02x reg %d sz %d", sub, reg, sizeof *this);
            }
        };

    } // namespace i2c

    namespace spi {
        struct Config {
            uint32_t addr =0;
            uint16_t ena =0;
            uint8_t mhz =0;
            Irq txIrq ={}, rxIrq ={};
            uint32_t dmaAddr =0;
            uint8_t dmaTx =0, dmaRx =0;
        };

        using Gpio = ::jeeh::spi::Gpio;

        template< typename DEV, Config const& CFG >
        struct Dev {
            static constexpr IoReg<CFG.addr> SPI {};
            DmaConfig<CFG.dmaAddr,CFG.dmaTx,CFG.dmaRx> dma;

            Pin sub;

            Dev (Pin s) : sub (s) {
            }

            void read (uint8_t reg) const {
                logf("sub %02x reg %d sz %d", sub, reg, sizeof *this);
            }
        };

    } // namespace spi

    constexpr auto i2cConf = i2c::Config{ I2C1.ADDR };
    i2c::Dev<i2c::Gpio,i2cConf> fram (0x50);

    constexpr auto spiConf = spi::Config{ SPI1.ADDR };
    spi::Dev<spi::Gpio,spiConf> sram (Pin {"A4"});

    void run () {
        logf("i2c %08x", fram.I2C.ADDR);
        logf("spi %08x", sram.SPI.ADDR);
        fram.read(123);
        sram.read(234);
    }
} // namespace ni2c

#if POLLED
i2c::Poll<I2C1.ADDR> i2cBus (ena::I2C1, 170);
#else
i2c::Gpio i2cBus;
#endif

i2c::Dev fram (i2cBus, 0x50);

Pin trigger ("B4","P");

constexpr auto N = 5;

void read32 (uint16_t addr, uint8_t* ptr) {
    trigger = 0;
    auto ok = fram.read16be(addr, ptr, N);
    assert(ok);
}

void write32 (uint16_t addr, uint8_t const* ptr) {
    trigger = 1;
    auto ok = fram.write16be(addr, ptr, N);
    assert(ok);
}

int main () {
    initBoard();
    switch (dog::resetCause()) {
        default: logf("reset cause?"); break;
        case 0:  logf("watchdog"); break;
        case 1:  logf("power up"); break;
        case 2:  logf("system reset"); break;
    }
    //dog::init(2);  // will fire approx. every 2 secs

    i2cBus.init("B7:OUL4,A15", 1000); // sda scl
    i2c::detect(i2cBus);
#if 1
    i2cBus.deinit();
    ni2c::run();
    while (true) { cycles::msBusy(500); led.toggle(); }
#endif

    auto seq = 0;
    while (true) {
        //dog::kick();
        logf("#%d", ++seq);

        uint8_t buf [N];
        memset(buf, 0xEE, sizeof buf);
        for (auto i = 0; i < 3; ++i)
            write32(N*i, buf);
        cycles::usBusy(40);

        for (auto i = 0; i < 3; ++i) {
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
            read32(N*i+2, buf);
            //logDump(buf, 16);
            logf("%02x %02x ... %02x %02x", buf[0], buf[1], buf[N-2], buf[N-1]);
        }

        fram.read16be(0, buf, 1);
        fram.read16be(N, buf+1, 1);
        fram.read16be(2*N+3, buf+2, 1);
        logf("%02x %02x %02x", buf[0], buf[1], buf[2]);

        cycles::msBusy(1000);
        led.toggle();
    }
}
