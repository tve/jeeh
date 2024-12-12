namespace jeeh::spi {

struct Config {
    char const* pins;           // gpio
    uint32_t base =0;           // poll
    uint16_t ena =0;
    uint8_t mhz =0;
    uint32_t dmaBase =0;        // sync
    uint8_t dmaIdx =0, dmaTs =0, dmaRs =0, dmaTc =0, dmaRc =0;
    Irq txIrq ={}, rxIrq ={};
};

template< Config const& C >
struct Gpio {
    using IoSize = uint16_t;

    Pin mosi, miso, sclk, nsel; // pin definitions must be kept in this order
    uint16_t rate =0;
    uint8_t cpol =0;

    void init (int khz =10'000) {
        Pin::config(C.pins, &mosi, 4);
        Pin::config(":HP,:U,:HP,", &mosi, 4);
        rate = khz < 100 ? khz : SystemCoreClock/khz/200'000; // TODO
        sclk = cpol;
    }

    void deinit () {
        Pin::config(":F,,,:U", &mosi, 4);
    }

    void select (Pin sel) {
        nsel = sel;
        nsel = 1; // start with nsel high
        nsel.mode("HP");
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

    int ioRequest (uint16_t m, uint8_t* p, uint16_t n) const {
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
    void checkStart (uint16_t m) const {
        if ((m & IO_START) && nsel.isValid()) {
            hold();
            nsel = 0;
            hold();
        }
    }

    void checkStop (uint16_t m) const {
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

template< Config const& C >
struct Poll : Gpio<C> {
    using BASE = Gpio<C>;

    static constexpr IoReg<C.base> SPI {};
    enum { CR1=0x00, CR2=0x04, SR=0x08, DR=0x0C }; // SPI regs

    void init (int khz =10'000) {
        Pin::config(C.pins, &(BASE::mosi), 4);

        int clk = SystemCoreClock / 1'000;
        while (clk > 1000 * C.mhz)
            clk /= 2;
        auto div = 0; // determine clock divider
        while ((clk >> (div+1)) > khz)
            ++div;
        assert(div <= 7);

        RCC(C.ena, 1) = 1;
        SPI[CR1] = (div<<3) | (1<<2); // BD MSTR
#if STM32F1 | STM32F4 | STM32L0
        SPI[CR2] = (1<<2); // SSOE
#else
        SPI[CR2] = (1<<12) | (7<<8) | (1<<2); // FRXTH DS SSOE
#endif
        SPI[CR1](6) = 1; // SPE
    }

    void deinit () {
        RCC(C.ena, 1) = 0;
        BASE::deinit();
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

    int ioRequest (uint16_t m, uint8_t* p, uint16_t n) const {
        uint8_t r = 0;
        BASE::checkStart(m);
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
        BASE::checkStop(m);
        return m & IO_LAST ? r : n;
    }
};

template< Config const& C >
struct Sync : Poll<C> {
    using BASE = Poll<C>;
    using BASE::SPI;

    static constexpr dma::DmaConfig<Config,C> dma {};

    void init (int khz =10'000) {
        BASE::init(khz);
        SPI[BASE::CR2](0,2) = 0b11; // TXDMAEN RXDMAEN
        dma.init(C.base + BASE::DR, C.base + BASE::DR);
    }

    void deinit () {
        SPI[BASE::CR2](0,2) = 0; // ~TXDMAEN ~RXDMAEN
        dma.deinit();
        BASE::deinit();
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

    int ioRequest (uint16_t m, uint8_t* p, uint16_t n) const {
        assert(!Task::pendingIrq());
        if (startReq(m, p, n)) {
            while (!Task::pendingIrq())
                asm ("wfe");
            dma.completed();
            assert(!dma.isRunning());
            Task::irqClear(C.txIrq);
            Task::irqClear(C.rxIrq);
        }
        return finishReq(m, p, n);
    }

protected:
    bool startReq (uint16_t m, void* p, uint16_t n) const {
        assert(!SPI[BASE::SR](7)); // ~BSY
        assert(SPI[BASE::SR](11,2) == 0); // FTLVL
        assert(SPI[BASE::SR](9,2) == 0); // FRLVL

        BASE::checkStart(m);
        if (n == 0)
            return false;
        // TODO try to get read+write working, replacing same buffer
        if (m & IO_WRITE)
            dma.txStart(p, n);
        else {
            dma.rxStart(p, n);
            SPI[BASE::CR1](10) = 1; // RXONLY
            SPI[BASE::CR1](6) = 1; // SPE
        }
        return true;
    }

    uint16_t finishReq (uint16_t m, void* p, uint16_t n) const {
        uint8_t r = 0;
        if (n > 0) {
            if (m & IO_READ) {
                cache::inval(p, n);
                SPI[BASE::CR1](10) = 0; // ~RXONLY
            }

            while (SPI[BASE::SR](11,2) != 0) {} // FTLVL
            while (SPI[BASE::SR](7)) {} // BSY

            while (SPI[BASE::SR](9,2) != 0) // FRLVL
                r = +SPI.byte(BASE::DR);
        }
        BASE::checkStop(m);
        return m & IO_LAST ? r : n;
    }
};

template< Config const& C >
struct Async : Sync<C>, Task {
    using BASE = Sync<C>;

    enum TAG { START, REQUEST, DONE };

    uint8_t init (int khz =10'000) {
        BASE::init(khz);
        return Task::init();
    }

    void setReply (Event out) const {
        pend = out;
    }

    int ioRequest (IoReq const* v, uint32_t n) const {
        if (!pend)
            return BASE::ioRequest(v, n); // use sync version
        reqs = v;
        num = n;
        send({ tId, REQUEST });
        return 0;
    }

    int ioRequest (uint16_t m, uint8_t* p, uint16_t n) const {
        curr = { m, n, p };
        return ioRequest(&curr, 1);
    }

    void irqDma () {
        auto f = BASE::dma.completed();
        assert(f > 0);
        trigger(DONE);
    }

private:
    mutable IoReq curr ={ 0, 0, nullptr };
    mutable IoReq const* reqs;
    mutable int num =0;
    mutable Event pend;

    Event process (Event in, Event out) override {
        assert(!out); // should use setReply instead
        switch (in.eTag) {
            case START:
                break;
            case REQUEST:
                irqEnable(C.txIrq);
                irqEnable(C.rxIrq);
                while (--num >= 0) {
                    curr = *reqs++;
                    if (BASE::startReq(curr.mode, curr.ptr, curr.len))
                        break; // transfer started, wait for DONE trigger
            case DONE:         // this jumps back into the transfer loop!
                    pend.eVal = BASE::finishReq(curr.mode, curr.ptr, curr.len);
                }
                irqDisable(C.txIrq);
                irqDisable(C.rxIrq);
                out = take(pend);
                break;
            default:
                fail();
        }
        return out;
    }
};

} // namespace jeeh::spi
