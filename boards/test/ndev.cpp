#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/uart.h>
#include "xi2c.h"
#include "xspi.h"
using namespace jeeh;
#include "defs.h"

namespace ndev { //-------------------------------------------------------------

namespace i2c {
    struct Config {
        using Addr = uint8_t;
        uint32_t base =0;
        uint16_t ena =0;
        uint8_t mhz =0;
        Irq evIrq ={}, erIrq ={}, txIrq ={}, rxIrq ={};
        uint32_t dmaAddr =0;
        uint8_t dmaTx =0, dmaRx =0;
    };

    using Gpio = ::jeeh::i2c::Gpio;

} // namespace i2c

namespace spi {
    struct Config {
        using Addr = Pin;
        uint32_t base =0;
        uint16_t ena =0;
        uint8_t mhz =0;
        Irq txIrq ={}, rxIrq ={};
        uint32_t dmaAddr =0;
        uint8_t dmaTx =0, dmaRx =0;
    };

    struct Gpio {
        Pin mosi, miso, sclk, nsel; // pin definitions must be kept in this order
        uint16_t rate =0;
        uint8_t cpol =0;

        void init (char const* desc, int khz =10'000) {
            Pin::config(desc, &mosi, 4);
            Pin::config(":HP,:U,:HP,", &mosi, 4);
            sclk = cpol;
            disable(); // start with NSEL high

            rate = khz < 1000 ? khz : SystemCoreClock/khz/200'000; // TODO
        }

        void deinit () {
            Pin::config(":F,,,:U", &mosi, 4);
        }

        void enable () const { hold(); nsel = 0; hold(); }
        void disable () const { hold(); nsel = 1; hold(); }

        int rwByte (int v) const {
            auto r = 0;
            for (auto i = 0; i < 8; ++i) {
                mosi = v >> 7;
                v <<= 1;
                hold();
                sclk = ~cpol;
                hold();
                r = (r<<1) | miso;
                sclk = cpol;
            }
            return r;
        }

        uint8_t transfer (uint8_t w, uint8_t* p, uint16_t n) const {
            uint8_t r = 0;
            auto q = (uint8_t*) p;
            if (w)
                for (auto i = 0U; i < n; ++i)
                    r = rwByte(*q++); // return last byte from reply
            else
                for (auto i = 0U; i < n; ++i)
                    *q++ = rwByte(0);
            return r;
        }

    private:
        void hold () const {
            for (volatile int i = rate; i >= 0; ) i = i-1;
        }
    };

} // namespace spi

template< typename D, typename T, T const& CFG >
struct Dev {
    static constexpr IoReg<CFG.base> DEV {};
    DmaConfig<CFG.dmaAddr,CFG.dmaTx,CFG.dmaRx> dma;

    using Addr = typename T::Addr;
    Addr sub;

    Dev (Addr s) : sub (s) {
    }

    void read (uint8_t reg) const {
        logf("sub %02x reg %d sz %d", sub, reg, sizeof *this);
    }
};

constexpr auto i2cConf = i2c::Config{ I2C1.ADDR };
Dev<i2c::Gpio,i2c::Config,i2cConf> fram (0x50);

constexpr auto spiConf = spi::Config{ SPI1.ADDR };
Dev<spi::Gpio,spi::Config,spiConf> sram (Pin {"A4"});

void run () {
    logf("i2c %08x", fram.DEV.ADDR);
    logf("spi %08x", sram.DEV.ADDR);
    fram.read(123);
    sram.read(234);
}

} // namespace ndev ------------------------------------------------------------

int main () {
    initBoard();
    ndev::run();
    while (true) { cycles::msBusy(500); led.toggle(); }
}
