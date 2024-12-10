namespace jeeh::i2c {

struct Config {
    char const* pins;           // gpio
    uint32_t base =0;           // poll
    uint16_t ena =0;
    uint8_t mhz =0;
    uint32_t dmaBase =0;        // sync
    uint8_t dmaIdx =0, dmaTs =0, dmaRs =0, dmaTc =0, dmaRc =0;
    Irq txIrq ={}, rxIrq ={}, evIrq ={}, erIrq ={};
};

template< typename I2C >
void detect (I2C& bus) {
    for (auto i = 0; i < 128; i += 16) {
        printf("%02x:", i);
        for (auto j = 0; j < 16; ++j) {
            uint8_t addr = i + j;
            if (0x08 <= addr && addr <= 0x77) {
                bus.select(addr);
                auto ack = bus.write(nullptr, 0) >= 0;
                printf(ack ? " %02x" : " --", addr);
            } else
                printf("   ");
        }
        printf("\n");
    }
}

template< Config const& C >
struct Gpio {
    using IoSize = uint8_t;

    Pin sda, scl; // pin definitions must be kept in this order
    uint8_t addr =0;
    uint16_t rate;

    void init (int khz =400) {
        Pin::config(C.pins, &sda, 2);
        Pin::config(":OU,", &sda, 2);

        sda = 1;
        scl = 1;

        // this is merely a wild estimate for the countdown needed in hold()
        // values < 100 will override to define a specific countdown instead
        rate = khz <= 0 ? -khz : SystemCoreClock/khz/100'000 + 1;
    }

    void deinit () {
        Pin::config(":F,", &sda, 2);
    }

    void select (uint8_t a) {
        addr = a;
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

    int ioRequest (uint16_t m, uint8_t* p, uint8_t n) const {
        bool ack = true;

        if (m & IO_START) {
            sclLo();
            sclHi();
            sda = 0;
            ack = wrByte(2*addr + ((m & IO_READ) != 0));
        }

        if (ack) {
            if (m & IO_WRITE)
                for (auto i = 0; ack && i < n; ++i)
                    ack = wrByte(*p++);
            else
                for (auto i = 0; i < n; ++i)
                    *p++ = rdByte(i == n-1);
        }

        if ((m & IO_STOP) || !ack) {
            sda = 0;
            sclHi();
            sda = 1;
            hold();
        }

        return ack ? n : -1;
    }

private:
    int rdByte (bool last) const {
        uint8_t data = 0;
        for (auto mask = 0x80; mask != 0; mask >>= 1) {
            sclHi();
            if (sda)
                data |= mask;
            sclLo();
        }
        sda = last;
        sclHi();
        sclLo();
        sda = 1;
        return data;
    }

    bool wrByte (uint8_t data) const {
        sclLo();
        for (auto mask = 0x80; mask != 0; mask >>= 1) {
            sda = (data & mask) != 0;
            sclHi();
            sclLo();
        }
        sda = 1;
        sclHi();
        hold();
        bool ack = !sda;
        sclLo();
        return ack;
    }

    void hold () const {
        for (volatile int i = rate; i >= 0; ) i = i-1;
    }
    void sclLo () const {
        hold();
        scl = 0;
    }
    void sclHi () const {
        hold();
        scl = 1;
        // wait a limited amount of time in case of clock stretching
        for (auto i = SystemCoreClock>>15; scl == 0 && i > 0; --i) {}
    }
};

template< Config const& C >
struct Poll : Gpio<C> {
    using BASE = Gpio<C>;
    using BASE::addr;

    static constexpr IoReg<C.base> I2C {};
#if STM32F4
    enum { CR1=0x00,CR2=0x04,DR=0x10,SR1=0x14,SR2=0x18,
           CCR=0x1C,TRISE=0x20 };
#else
    enum { CR1=0x00,CR2=0x04,TIMINGR=0x10,TIMOUTR=0x14,
           ISR=0x18,ICR=0x1C,RXDR=0x24,TXDR=0x28 };
#endif

    Pin sda, scl; // pin definitions must be kept in this order

    void init (int khz =400) {
        Pin::config(C.pins, &sda, 2);

        if (!sda) { // reset the I2C bus if SDA is stuck low
            scl.mode("OU");
            sda.mode("OU");
            for (auto i = 0; i < 32; ++i) {
                scl.toggle();
                cycles::usBusy(10);
            }
            scl = 1;
            assert(sda); // should now be unstuck
            Pin::config(C.pins, &sda, 2);
        }

        RCC(C.ena,1) = 1;
#if STM32F4
        I2C[CR1](15) = 1; // SWRST
        I2C[CR1](15) = 0; // ~SWRST
#endif
        setTiming(khz);
        I2C[CR1] = 1; // PE
    }

    void deinit () {
        Pin::config(":F,", &sda, 2);
        I2C[CR1](0) = 0; // ~PE
        RCC(C.ena, 1) = 0;
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

#if STM32F4
    bool xtransfer (uint8_t a, uint8_t m, uint8_t* p, uint8_t n) const {
        auto waitFor = [](uint8_t bit) {
            while (!I2C[SR1](bit))
                if (I2C[SR1] & 0x4D00) { // TIMEOUT OVR AF BERR
                    I2C[SR1] = 0;
                    return false;
                }
            return true;
        };

        if (m != W2) {
            I2C[CR1](10) = 1; // ACK
            I2C[CR1](8) = 1; // START
            while (!I2C[SR1](0)) {} // ~SB
        }
        switch (m) {
            case R1:
            case W1:
                I2C[DR] = a<<1;
                if (!waitFor(1)) // ADDR
                    return false;
                (void) +I2C[SR2];
                [[fallthrough]];
            case W2:
                if (n > 0) {
                    do {
                        if (!waitFor(7)) // TXE
                            return false;
                        I2C[DR] = *p++;
                    } while (--n > 0);
                    if (!waitFor(2)) // BTF
                        return false;
                }
                break;
            case R2:
                I2C[DR] = (a<<1)+1;
                if (!waitFor(1)) // ADDR
                    return false;
                I2C[CR1](10) = n > 1; // ACK if multiple
                (void) +I2C[SR2];
                do {
                    if (n == 1) { // about to read last byte
                        I2C[CR1](10) = 0; // ~ACK
                        I2C[CR1](9) = 1; // STOP
                    }
                    while (!I2C[SR1](6)) {} // ~RXNE
                    *p++ = I2C[DR];
                } while (--n > 0);
                break;
            default:
                fail();
        }
        if (m == W2)
            I2C[CR1](9) = 1; // STOP
        return true;
    }
#endif

    int ioRequest (uint32_t m, uint8_t* p, uint8_t n) const {
        startReq(m, n);
        while ((I2C[ISR] & 0x10F0) == 0) // ~TIMEOUT ~TCR ~TC ~STOPF ~NACKF
            if (I2C[ISR](2)) // RXNE
                *p++ = I2C[RXDR];
            else if (I2C[ISR](1)) // TXIS
                I2C[TXDR] = *p++;
        return finishReq(m, n);
    }

protected:
    void startReq (uint8_t m, uint8_t n) const {
        I2C[CR2] = (((m & IO_STOP) != 0)  << 25) // AUTOEND
                 | (((m & IO_MORE) != 0)  << 24) // RELOAD
                 |                     (n << 16) // NBYTES
                 | (((m & IO_START) != 0) << 13) // START
                 | (((m & IO_READ) != 0)  << 10) // RD_WRN
                 |                  (addr << 1); // SADD
    }

    int finishReq (uint16_t m, uint16_t n) const {
        if (m & IO_STOP)
            while (I2C[ISR](15)) {} // BUSY
        auto ok = !I2C[ISR](12) && !I2C[ISR](4); // ~TIMEOUT ~NACKF
        I2C[ICR] = I2C[ISR];
        return ok ? n : -1;
    }

private:
    void setTiming (uint32_t khz) {
        assert(khz > 0);
#if STM32F4
        if (khz < 10'000) {
            auto div = (1000 * C.mhz) / khz;
            I2C[CR2] = C.mhz;
            I2C[TRISE] = div/4; // seems to work well
            I2C[CCR] = khz <= 100 ? div/2 :
                       khz <= 400 ? (2<<14) | div/3 :
                                    (3<<14) | div/25;
        } else { // custom rate settings
            I2C[CR2] = khz>>24;
            I2C[TRISE] = (uint8_t) (khz>>16);
            I2C[CCR] = (uint16_t) khz;
        }
#else
        if (khz < 10'000) {
            auto div = (1000 * C.mhz) / khz;
            auto presc = div/256;
            assert(presc < 16);
            div /= presc+1;
            khz = (presc<<28) | (5<<20) | (1<<16) | (div/4<<8) | (3*div/4<<0);
        } // else custom rate settings
        I2C[TIMINGR] = khz;

        // 25 ms timeout is approx 12x I2C clock in Mhz (i.e. sysclk/prescaler)
        // see table 394, p.1909 in RM0440 r8 for some suggested values
        // FIXME should this be C.mhz iso SystemCoreClock ?
        auto t = 12 * ((SystemCoreClock>>20) / ((khz>>28) + 1));
        assert(t < 4096);
        I2C[TIMOUTR] = (1<<15) | t; // TIMOUTEN
#endif
    }
};

template< Config const& C >
struct Sync : Poll<C> {
    using BASE = Poll<C>;
    using BASE::I2C;

    static constexpr dma::DmaConfig<Config,C> dma {};

    void init (uint32_t khz =400) {
        BASE::init(khz);
        I2C[BASE::CR1](14,2) = 0b11; // RXDMAEN TXDMAEN
        dma.init(C.base + BASE::TXDR, C.base + BASE::RXDR);
        SCB[0x10](4) = 1; // SEVONPEND
    }

    void deinit () {
        I2C[BASE::CR1](14,2) = 0; // ~RXDMAEN ~TXDMAEN
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

    int ioRequest (uint32_t m, uint8_t* p, uint8_t n) const {
        assert(!Task::pendingIrq());
        startReq(m, p, n);
        while (!Task::pendingIrq())
            asm ("wfe");
        dma.done();
        dma.completed();
        assert(!dma.isRunning());
        auto r = finishReq(m, p, n);
        Task::irqClear(C.evIrq);
        Task::irqClear(C.erIrq);
        Task::irqClear(C.txIrq);
        Task::irqClear(C.rxIrq);
        return r;
    }

protected:
    void startReq (uint16_t m, void* p, uint8_t n) const {
        // must set up DMA before START, see 33.4.16, p.1003 in RM0393 v2
        if (m & IO_WRITE)
            dma.txStart(p, n);
        else
            dma.rxStart(p, n);

        BASE::startReq(m, n);
        I2C[BASE::CR1](4,4) = 0b1111; // ERRIE TCIE STOPIE NACKIE
    }

    int finishReq (uint16_t m, void* p, uint8_t n) const {
        I2C[BASE::CR1](4,4) = 0; // ~ERRIE ~TCIE ~STOPIE ~NACKIE
        if (m & IO_READ)
            cache::inval(p, n);
        return BASE::finishReq(m, n);
    }
};

#if 0
template< uint32_t A, uint32_t D, int T, int R >
struct Async : Sync<A,D,T,R>, Task {
    using BASE = Sync<A,D,T,R>;
    using BASE::Sync;

    enum TAG { START, RXDONE, TXDONE };

    Event pending;

    uint8_t init (char const* defs, uint32_t khz =400) {
        BASE::init(defs, khz);
        irqEnable(cfg.evIrq);
        //irqEnable(cfg.erIrq);
        return Task::init();
    }

    void deinit () {
        irqDisable(cfg.evIrq);
        //irqDisable(cfg.erIrq);
        BASE::deinit();
    }

    // async version, started from a msg
    void start (uint8_t a, uint8_t m, uint8_t* p, uint16_t n, Event out) {
        assert(n > 0);
        pending = out;
        BASE::startReq(a, n, p, n);
    }

#if 0
    void finish () {
        auto mp = msgs.pull();
        if (mp == nullptr)
            return;
        mp->mLen = finishReq(mp->mLen >> 8, mp->mPtr, (uint8_t) mp->mLen);
        reply(mp);
        if (!msgs.isEmpty())
            startAsync(*msgs.first());
    }

    void startAsync (Event m) {
        uint8_t mode = m.mLen >> 8, len = m.mLen;
        startReq(m.mTag, mode, m.mPtr, len);
        if (mode == BASE::W1 && len == 0)
            finish(); // this may be recursive
    }
#endif

    void irqI2c () {
        BASE::I2C[BASE::CR1](4,3) = 0; // ~TCIE ~STOPIE ~NACKIE
        trigger(RXDONE); // TODO TXDONE?
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
#endif

} // namespace jeeh
