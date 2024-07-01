// see examples/spi/{endure,spif}.cpp

namespace jeeh::spi {

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

    Config const cfg;
    Pin nsel;

    Poll (uint16_t e, uint8_t f) : cfg { e, f } {}

    void init (char const* defs, int khz) const {
        Pin pins [4]; // mosi, miso, nclk, nsel
        Pin::config(defs, pins, sizeof pins);
        nsel = pins[3];
        disable(); // start with NSEL high

        auto div = 0; // determine clock divider
        while ((1000*cfg.mhz >> (div+1)) > khz)
            ++div;
        assert(div <= 7);

        RCC(cfg.ena, 1) = 1;
        SPI[CR1] = (div<<3) | (1<<2); // BD MSTR
#if STM32F1 | STM32L0
        SPI[CR2] = (1<<2); // SSOE
#else
        SPI[CR2] = (1<<12) | (7<<8) | (1<<2); // FRXTH DS SSOE
#endif
        SPI[CR1](6) = 1; // SPE
    }

    void deinit () { RCC(cfg.ena, 1) = 0; }

    void enable () const { nsel.clear(); }
    void disable () const { nsel.set(); }

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

// DMA version, either sync-wfe or async (i.e. msgs sent to this device)
template< uint32_t A, uint32_t D, int T, int R >
struct Sync : Poll<A>, Device {
    using BASE = Poll<A>;

    static constexpr IoReg<A> SPI {};

    struct Config : BASE::Config {
        Irq txIrq, rxIrq;
        uint8_t Xdma, XtxReq, XrxReq; // 0-based
    };

    DmaConfig<D,T,R> const dma;
    Config const cfg;

    Sync (Config const& c)
        : BASE (c.ena, c.mhz), Device ('S'),
          dma { c.Xdma, c.XtxReq, c.XrxReq }, cfg (c) {}

    void init (char const* defs, int khz) {
        BASE::init(defs, khz);
        SPI[BASE::CR2](0,2) = 0b11; // TXDMAEN RXDMAEN

        // peripheral address config and interrupt vector setup
        dma.init(A + BASE::DR, A + BASE::DR);

        irqInstall((uint8_t) cfg.txIrq);
        irqInstall((uint8_t) cfg.rxIrq);
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

protected:
    constexpr static auto LEN_BITS = 14, LEN_MASK = (1<<(LEN_BITS+1)) - 1;

private:
    Chain msgs;

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

    // async version, started from a msg
    void start (Message& m) override {
        if (!msgs.append(m))
            startAsync(m);
    }

    void finish () override {
        auto mp = msgs.pull();
        if (mp == nullptr)
            return;
        mp->mLen = finishReq(mp->mTag, mp->mPtr, mp->mLen);
        reply(mp);
        if (!msgs.isEmpty())
            startAsync(*msgs.first());
    }

    void startAsync (Message& m) {
        if (m.mLen > 0)
            startReq(m.mTag, m.mPtr, m.mLen);
        else
            finish(); // this may be recursive
    }

    bool interrupt (int) override {
        if (!dma.completed())
            fail();
        if (dma.isRunning())
            return false; // other channel still in progress

        return !msgs.isEmpty();
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Call : Sync<A,D,T,R> {
    using BASE = Sync<A,D,T,R>;
    using BASE::Sync; // constructor

    // async version, dma with sys::call
    uint8_t transfer (uint8_t w, uint8_t* p, uint16_t n) const {
        Message msg { BASE::dId, w, n, (uint8_t*) p };
        sys::call(msg); // async with thread suspend
        return msg.mLen;
    }
};

} // namespace jeeh
