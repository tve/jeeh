namespace jeeh::spi {

struct Gpio {
    Pin mosi, miso, sclk, nsel; // pin definitions must be kept in this order
    uint16_t rate =0;
    uint8_t cpol =0;

    void init (char const* desc, int khz =10'000) {
        Pin::config(desc, &mosi, 4);
        Pin::config(":HP,:U,:HP,", &mosi, 4);
        sclk = cpol;
        ioRequest(IO_STOP); // start with nsel high
        rate = khz < 1000 ? khz : SystemCoreClock/khz/200'000; // TODO
    }

    void deinit () {
        Pin::config(":F,,,:U", &mosi, 4);
    }

    template< uint32_t N >
    int ioRequest (IoReq const (&v) [N]) const {
        return ioRequest(v, N);
    }

    int ioRequest (IoReq const* v, uint32_t n) const {
        int r = 0;
        for (auto i = 0U; i < n; ++i) {
            auto& t = v[i];
            r = ioRequest(t.mode, t.ptr, t.len);
            if (r < 0)
                break;
        }
        return r;
    }

    int ioRequest (uint32_t m, uint8_t* p =nullptr, uint16_t n =0) const {
        uint8_t r = 0;
        checkStart(m);
        if (m & IO_WRITE)
            for (auto i = 0U; i < n; ++i)
                r = rwByte(*p++); // return last byte from reply
        else
            for (auto i = 0U; i < n; ++i)
                *p++ = rwByte(0);
        checkStop(m);
        return m & IO_LAST ? r : n;
    }

protected:
    void checkStart (uint32_t m) const {
        if ((m & IO_START) && nsel.isValid()) {
            hold();
            nsel = 0;
            hold();
        }
    }

    void checkStop (uint32_t m) const {
        if ((m & IO_STOP) && nsel.isValid()) {
            hold();
            nsel = 1;
            hold();
        }
    }

private:
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

    void hold () const {
        for (volatile int i = rate; i >= 0; ) i = i-1;
    }
};

// polled H/W version (see spi::Gpio for bit-banged version)
template< uint32_t A >
struct Poll : Gpio {
    using BASE = Gpio;

    static constexpr IoReg<A> SPI {};
    enum { CR1=0x00, CR2=0x04, SR=0x08, DR=0x0C }; // SPI regs

    struct Config {
        uint16_t ena;
        uint8_t mhz;
    };

    Config const cfg;

    Poll (uint16_t e, uint8_t f) : cfg { e, f } {}

    void init (char const* defs, int khz =10'000) {
        Pin::config(defs, &mosi, 4);
        ioRequest(IO_STOP); // start with nsel high

        int clk = SystemCoreClock / 1'000;
        while (clk > 1000 * cfg.mhz)
            clk /= 2;
        auto div = 0; // determine clock divider
        while ((clk >> (div+1)) > khz)
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
        RCC(cfg.ena, 1) = 0;
        BASE::deinit();
    }

    int ioRequest (uint32_t m, uint8_t* p =nullptr, uint16_t n =0) const {
        uint8_t r = 0;
        checkStart(m);
        if (n > 0) {
            if (m & IO_WRITE) {
                SPI.byte(DR) = *p++;
                while (--n != 0) {
                    while (!SPI[SR](1)) {} // ~TXE
                    SPI.byte(DR) = *p++;
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
                    *p++ = SPI.byte(DR);
                }
                while (!SPI[SR](0)) {} // ~RXNE
                *p = SPI.byte(DR);
            }
        }
        checkStop(m);
        return m & IO_LAST ? r : n;
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Sync : Poll<A> {
    using BASE = Poll<A>;

    static constexpr IoReg<A> SPI {};

    struct Config : BASE::Config {
        Irq txIrq, rxIrq;
        DmaConfig<D,T,R> dma;
    };

    Config const cfg;

    Sync (Config const& c) : BASE (c.ena, c.mhz), cfg (c) {}

    void init (char const* defs, int khz =10'000) {
        BASE::init(defs, khz);
        SPI[BASE::CR2](0,2) = 0b11; // TXDMAEN RXDMAEN
        cfg.dma.init(A + BASE::DR, A + BASE::DR);
        SCB[0x10](4) = 1; // SEVONPEND
    }

    void deinit () {
        SPI[BASE::CR2](0,2) = 0; // ~TXDMAEN ~RXDMAEN
        cfg.dma.deinit();
        BASE::deinit();
    }

    uint32_t ioRequest (uint32_t m, uint8_t* p =nullptr, uint16_t n =0) const {
        uint8_t r = 0;
        BASE::checkStart(m);
        if (n > 0) {
            startReq(m & 1, p, n);
            while (true) {
                if (cfg.dma.completed() == 0)
                    asm ("wfe");
                if (!cfg.dma.isRunning())
                    break;
            }
//logf("12");
            Task::irqClear(cfg.txIrq);
            Task::irqClear(cfg.rxIrq);
            r = finishReq(m & 1, p, n);
        }
        BASE::checkStop(m);
        return m & IO_LAST ? r : n;
    }

protected:
    void startReq (bool w, void* p, uint16_t n) const {
        assert(n > 0);

        assert(!SPI[BASE::SR](7)); // ~BSY
        assert(SPI[BASE::SR](11,2) == 0); // FTLVL
        assert(SPI[BASE::SR](9,2) <= 1); // FRLVL

        if (w)
            cfg.dma.txStart(p, n);
        else {
            SPI[BASE::CR1](6) = 0; // ~SPE
            SPI[BASE::CR1](10) = 1; // RXONLY
            cfg.dma.rxStart(p, n);
            SPI[BASE::CR1](6) = 1; // SPE needed to reaffirm?
        }
    }

    uint8_t finishReq (bool w, void* p, uint16_t n) const {
        if (!w) {
            SPI[BASE::CR1](10) = 0; // ~RXONLY
            while (SPI[BASE::SR](7)) {} // BSY
            cache::inval(p, n);
        }

        while (SPI[BASE::SR](7)) {} // BSY
        //while (SPI[BASE::SR](11,2) != 0) {} // FTLVL
        //while (SPI[BASE::SR](7)) {} // BSY

        assert(SPI[BASE::SR](11,2) == 0); // FTLVL
        //assert(SPI[BASE::SR](9,2) <= 1); // FRLVL

        uint8_t r;
        do
            r = SPI.byte(BASE::DR);
        while (SPI[BASE::SR](9,2) > 0); // FRLVL

        assert(SPI[BASE::SR](11,2) == 0); // FTLVL
        assert(SPI[BASE::SR](9,2) <= 1); // FRLVL

        return r;
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Async : Sync<A,D,T,R>, Task {
    using BASE = Sync<A,D,T,R>;
    using BASE::Sync, BASE::cfg;

    enum TAG { START, RXDONE, TXDONE };

    Event pending;

    uint8_t init (char const* defs, int khz =10'000) {
        BASE::init(defs, khz);
        irqEnable(cfg.txIrq);
        irqEnable(cfg.rxIrq);
        return Task::init();
    }

    void deinit () {
        irqDisable(cfg.txIrq);
        irqDisable(cfg.rxIrq);
        BASE::deinit();
    }

    // async version
    void start (uint8_t w, uint8_t* p, uint16_t n, Event out) {
        assert(n > 0);
        pending = out;
        BASE::startReq(w, p, n);
    }

    void irqDma () {
        auto f = cfg.dma.completed();
        if (f != 0)
            trigger(f == cfg.dma.TXDONE ? TXDONE : RXDONE);
    }

private:
    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                break;
            case RXDONE:
                pending.eVal = BASE::finishReq(false, nullptr, 0);
                reply(pending);
                break;
            case TXDONE:
                pending.eVal = BASE::finishReq(true, nullptr, 0);
                reply(pending);
                break;
            default:
                fail();
        }
        return out;
    }
};

} // namespace jeeh::spi
