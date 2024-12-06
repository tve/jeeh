namespace jeeh {

template< typename T >
struct Dev : T {

    template< uint32_t N >
    int ioRequest (IoReq const (&v) [N]) const {
        return ioRequest(v, N);
    }

    int ioRequest (IoReq const* v, uint32_t n) const {
        return ((T*) this)->ioRequest(v, n);
    }

    int ioRequest (uint32_t m, uint8_t* p =nullptr, uint16_t n =0) const {
        return ((T*) this)->ioRequest(m, p, n);
    }

    // simple reads and writes
    int read (void* p, uint16_t n) const {
        return ioRequest(IO_START|IO_READ|IO_STOP, (uint8_t*) p, n);
    }
    int write (void const* p, uint16_t n) const {
        return ioRequest(IO_START|IO_WRITE|IO_STOP, (uint8_t*) p, n);
    }

    // one byte address, single-byte data
    int readReg (uint8_t r) const {
        uint8_t v = 0;
        return readRegs(r, &v, 1) >= 0 ? v : -1;
    }
    int writeReg (uint8_t r, uint8_t v) const {
        return writeRegs(r, &v, 1);
    }

    // one byte address, read/write byte buffer
    int readRegs (uint8_t r, void* p, uint8_t n) const {
        IoReq const req [] = {
            { IO_START|IO_WRITE, 1, &r },
            { IO_READ|IO_STOP, n, (uint8_t*) p },
        };
        return ioRequest(req);
    }
    int writeRegs (uint8_t r, void const* p, uint8_t n) const {
        IoReq const req [] = {
            { IO_START|IO_WRITE|IO_MORE, 1, &r },
            { IO_WRITE|IO_STOP, n, (uint8_t*) p },
        };
        return ioRequest(req);
    }

    // two byte address, two-byte data, both big-endian
    int readReg16 (uint16_t r) const {
        uint16_t v = 0;
        return readRegs16(r, &v, 2) >= 0 ? __builtin_bswap16(v) : -1;
    }
    int writeReg16 (uint16_t r, uint16_t v) const {
        v = __builtin_bswap16(v); // send big-endian
        return writeRegs16(r, &v, 2);
    }

    // two byte big-endian address, read/write byte buffer
    int readRegs16 (uint16_t r, void* p, uint8_t n) const {
        r = __builtin_bswap16(r); // send big-endian
        IoReq const req [] = {
            { IO_START|IO_WRITE, 2, &r },
            { IO_READ|IO_STOP, n, (uint8_t*) p },
        };
        return ioRequest(req);
    }
    int writeRegs16 (uint16_t r, void const* p, uint8_t n) const {
        r = __builtin_bswap16(r); // send big-endian
        IoReq const req [] = {
            { IO_START|IO_WRITE|IO_MORE, 2, &r },
            { IO_WRITE|IO_STOP, n, (uint8_t*) p },
        };
        return ioRequest(req);
    }
};

} // namespace jeeh

namespace jeeh::spi {

struct Config {
    using Addr = Pin;
    uint32_t base =0;
    uint16_t ena =0;
    uint8_t mhz =0;
    Irq txIrq ={}, rxIrq ={};
    uint32_t dmaAddr =0;
    uint8_t dmaIdx =0, dmaTx =0, dmaRx =0;
};

template< Config const& C >
struct Gpio {
    using Config = spi::Config;

    Pin mosi, miso, sclk, nsel; // pin definitions must be kept in this order
    uint16_t rate =0;
    uint8_t cpol =0;

    void init (char const* desc, int khz =10'000) {
        Pin::config(desc, &mosi, 4);
        Pin::config(":HP,:U,:HP,", &mosi, 4);
        sclk = cpol;
        ioRequest(IO_STOP, nullptr, 0); // start with nsel high
        rate = khz < 1000 ? khz : SystemCoreClock/khz/200'000; // TODO
    }

    void deinit () {
        Pin::config(":F,,,:U", &mosi, 4);
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

    int ioRequest (uint32_t m, uint8_t* p, uint16_t n) const {
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
template< Config const& C >
struct Poll : Gpio<C> {
    using BASE = Gpio<C>;

    static constexpr IoReg<C.base> SPI {};
    enum { CR1=0x00, CR2=0x04, SR=0x08, DR=0x0C }; // SPI regs

    Poll () {}

    void init (char const* defs, int khz =10'000) {
        Pin::config(defs, &(BASE::mosi), 4);
        ioRequest(IO_STOP, nullptr, 0); // start with nsel high

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

    int ioRequest (uint32_t m, uint8_t* p, uint16_t n) const {
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

    DmaConfig<C.dmaAddr,C.dmaTx,C.dmaRx> dma;

    Sync () : dma { C.dmaIdx, C.dmaTx, C.dmaRx } {}

    void init (char const* defs, int khz =10'000) {
        BASE::init(defs, khz);
        SPI[BASE::CR2](0,2) = 0b11; // TXDMAEN RXDMAEN
        dma.init(C.base + BASE::DR, C.base + BASE::DR);
        SCB[0x10](4) = 1; // SEVONPEND
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

    uint32_t ioRequest (uint32_t m, uint8_t* p, uint16_t n) const {
        uint8_t r = 0;
        BASE::checkStart(m);
        if (n > 0) {
            startReq(m & 1, p, n);
            while (true) {
                if (dma.completed() == 0)
                    asm ("wfe");
                if (!dma.isRunning())
                    break;
            }
//logf("12");
            Task::irqClear(C.txIrq);
            Task::irqClear(C.rxIrq);
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
            dma.txStart(p, n);
        else {
            SPI[BASE::CR1](6) = 0; // ~SPE
            SPI[BASE::CR1](10) = 1; // RXONLY
            dma.rxStart(p, n);
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

template< Config const& C >
struct Async : Sync<C>, Task {
    using BASE = Sync<C>;
    using BASE::Sync, BASE::dma;

    enum TAG { START, RXDONE, TXDONE };

    Event pending;

    uint8_t init (char const* defs, int khz =10'000) {
        BASE::init(defs, khz);
        irqEnable(C.txIrq);
        irqEnable(C.rxIrq);
        return Task::init();
    }

    void deinit () {
        irqDisable(C.txIrq);
        irqDisable(C.rxIrq);
        BASE::deinit();
    }

    // async version
    void start (uint8_t w, uint8_t* p, uint16_t n, Event out) {
        assert(n > 0);
        pending = out;
        BASE::startReq(w, p, n);
    }

    void irqDma () {
        auto f = dma.completed();
        if (f != 0)
            trigger(f == dma.TXDONE ? TXDONE : RXDONE);
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
