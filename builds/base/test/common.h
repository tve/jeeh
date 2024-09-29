// Common code, used in all tests.

#include <unity.h>
#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
using namespace jeeh;

#undef assert
#define assert TEST_ASSERT

// tie printf and logf into Unity's output mechanism

extern "C" int _write (int, char* ptr, int len) {
    for (auto i = 0; i < len; ++i)
        putchar(ptr[i]);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

extern void allTests ();

int main () {
    fastClock();
    cycles::init();

    // adjust priorities before they might interfere with "real" IRQs
    SCB.byte(0x1F) = 0xFF; // irq #11: SVC
    SCB.byte(0x22) = 0xFF; // irq #14: PendSV

    UNITY_BEGIN();
    allTests();
    return UNITY_END();
}

// End of boilerplate, below is for temporary code, to be moved once ready.

namespace jeeh::spi {

struct Gpio {
    Pin mosi, miso, sclk, nsel; // pin definitions must be kept in this order
    uint16_t rate =0;
    uint8_t cpol =0;

    void init (char const* desc, int khz =10'000) {
        Pin::config(desc, &mosi, 4);
        disable(); // start with NSEL high
        Pin::config(":HP,:D,:HP,", &mosi, 4);
        sclk = cpol;

        rate = khz < 1000 ? khz : SystemCoreClock/khz/200'000; // TODO
    }

    void deinit () {
        Pin::config(":F,,,", &mosi, 4);
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
        for (volatile int i = rate; --i >= 0; ) {}
    }
};

// polled H/W version (see spi::Gpio for bit-banged version)
template< uint32_t A >
struct Poll {
    using ID = Pin;

    static constexpr IoReg<A> SPI {};
    enum { CR1=0x00, CR2=0x04, SR=0x08, DR=0x0C }; // SPI regs

    struct Config {
        uint16_t ena;
        uint8_t mhz;
    };

    Pin mosi, miso, sclk, nsel; // pin definitions must be kept in this order
    Config const cfg;

    Poll (uint16_t e, uint8_t f) : cfg { e, f } {}

    void init (char const* defs, int khz) {
        Pin::config(defs, &mosi, 4);
        disable(); // start with NSEL high

        auto div = 0; // determine clock divider
        while ((1000*cfg.mhz >> (div+1)) > khz)
            ++div;
        assert(div <= 7);

        RCC(cfg.ena, 1) = 1;
        SPI[CR1] = (div<<3) | (1<<2); // BD MSTR
#if STM32F1 | STM32F4 | STM32L0
        SPI[CR2] = (1<<2); // SSOE
#else
        SPI[CR2] = (1<<12) | (7<<8) | (1<<2); // FRXTH DS SSOE
#endif
        SPI[CR1](6) = 1; // SPE
    }

    void deinit () {
        Pin::config(":F,,,", &mosi, 4);
        RCC(cfg.ena, 1) = 0;
    }

    void enable () const { nsel = 0; }
    void disable () const { nsel = 1; }

    int rwByte (int v) const {
        SPI.byte(DR) = v;
        while (!SPI[SR](0)) {} // ~RXNE
        return SPI.byte(DR);
    }

    uint8_t transfer (uint8_t w, uint8_t* p, uint16_t n) const {
        uint8_t r = 0;
        if (n > 0) {
            auto q = (uint8_t*) p;
            if (w) {
                SPI.byte(DR) = *q++;
                while (--n != 0) {
                    while (!SPI[SR](1)) {} // ~TXE
                    SPI.byte(DR) = *q++;
                    while (!SPI[SR](0)) {} // ~RXNE
                    (void) +SPI.byte(DR);
                }
                while (!SPI[SR](0)) {} // ~RXNE
                r = SPI.byte(DR);
            } else {
                SPI.byte(DR) = 0;
                while (--n != 0) {
                    while (!SPI[SR](1)) {} // ~TXE
                    SPI.byte(DR) = 0;
                    while (!SPI[SR](0)) {} // ~RXNE
                    *q++ = SPI.byte(DR);
                }
                while (!SPI[SR](0)) {} // ~RXNE
                *q = SPI.byte(DR);
            }
        }
        return r;
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Sync : Poll<A>, Worker {
    using BASE = Poll<A>;

    enum TAG { DONE };
    static constexpr IoReg<A> SPI {};

    struct Config : BASE::Config {
        Irq txIrq, rxIrq;
    };

    Config const cfg;
    DmaConfig<D,T,R> const dma;

    Sync (Config const& c, DmaConfig<D,T,R> d, char const* name ="uart")
        : BASE (c.ena, c.mhz), Worker (name), cfg (c), dma (d) {}

    void init (char const* defs, int khz) {
        BASE::init(defs, khz);
        SPI[BASE::CR2](0,2) = 0b11; // TXDMAEN RXDMAEN

        // peripheral address config and interrupt vector setup
        dma.init(A + BASE::DR, A + BASE::DR);

        irqEnable(cfg.txIrq);
        irqEnable(cfg.rxIrq);
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma, 1) = 0; // may be shared

    // sync version, dma with wfe
    uint8_t transfer (uint8_t w, uint8_t* p, uint16_t n) const {
        if (n == 0)
            return 0;

        startReq(w, p, n);
        while (dma.isRunning())
            asm ("wfe");
        return finishReq(w, p, n);
    }

    void interrupt () {
        if (!dma.completed())
            fail();
        if (!dma.isRunning()) // other channel still in progress
            trigger(DONE);
    }

private:
    Event process (Event in, Event out, void* arg) override {
        (void) arg;
        switch (in.eTag) {
            case DONE:
                break;
            default:
                fail();
        }
        return out;
    }

    void startReq (bool w, void* p, uint16_t n) const {
        assert(n > 0);

        dma.txStart(p, n);
        if (!w)
            dma.rxStart(p, n);
    }

    uint8_t finishReq (bool w, void* p, uint16_t n) const {
        if (!w)
            cache::inval(p, n);
        uint8_t r;
        do
            r = SPI.byte(BASE::DR);
        while (SPI[BASE::SR](0)); // RXNE
        return r;
    }
};

} // namespace jeeh
