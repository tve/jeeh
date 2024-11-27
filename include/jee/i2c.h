namespace jeeh::i2c {

template< typename I2C >
struct Dev {
    I2C& bus;
    uint8_t id;

    Dev (I2C& b, uint8_t i) : bus (b), id (i) {}

    bool transfer (uint8_t m, void* p =nullptr, uint8_t n =0) const {
        return bus.transfer(id, m, p, n);
    }

    // one byte address, single-byte data
    int read (uint8_t r) const {
        uint8_t v = 0;
        return read(r, &v, 1) ? v : -1;
    }
    bool write (uint8_t r, uint8_t v) const {
        return write(r, &v, 1);
    }

    // one byte address, read/write byte buffer
    bool read (uint8_t r, void* p, uint8_t n) const {
        return transfer(bus.R1, &r, 1)
            && transfer(bus.R2, p, n);
    }
    bool write (uint8_t r, void const* p, uint8_t n) const {
        return transfer(bus.W1, &r, 1)
            && transfer(bus.W2, (void*) p, n);
    }

    // two byte address, two-byte data, both big-endian
    int read16be (uint16_t r) const {
        uint16_t v = 0;
        return read16be(r, &v, 2) ? (v<<8) | (v>>8) : -1;
    }
    bool write16be (uint16_t r, uint16_t v) const {
        v = (v<<8) | (v>>8); // send big-endian
        return write16be(r, &v, 2);
    }

    // two byte big-endian address, read/write byte buffer
    bool read16be (uint16_t r, void* p, uint8_t n) const {
        r = (r<<8) | (r>>8); // send big-endian
        return transfer(bus.R1, &r, 2)
            && transfer(bus.R2, p, n);
    }
    bool write16be (uint16_t r, void const* p, uint8_t n) const {
        r = (r<<8) | (r>>8); // send big-endian
        return transfer(bus.W1, &r, 2)
            && transfer(bus.W2, (void*) p, n);
    }
};

template< typename I2C >
void detect (I2C& bus) {
    for (auto i = 0; i < 128; i += 16) {
        printf("%02x:", i);
        for (auto j = 0; j < 16; ++j) {
            uint8_t addr = i + j;
            if (0x08 <= addr && addr <= 0x77) {
                Dev dev { bus, addr };
                bool ack = dev.transfer(bus.W1) && dev.transfer(bus.W2);
                printf(ack ? " %02x" : " --", addr);
            } else
                printf("   ");
        }
        printf("\n");
    }
}

struct Gpio {
    Pin sda, scl; // pin definitions must be kept in this order
    uint16_t rate;

    void init (char const* desc, uint32_t khz =400) {
        Pin::config(desc, &sda, 2);
        Pin::config(":OUL,", &sda, 2);

        sda = 1;
        scl = 1;

        // this is merely a wild estimate for the countdown needed in hold()
        // values < 100 will override to define a specific countdown instead
        rate = khz < 100 ? khz : SystemCoreClock/khz/200'000 + 1;
    }

    void deinit () {
        Pin::config(":F,", &sda, 2);
    }

    enum { R1, R2, W1, W2 };

    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        bool ack = true;

        if (m == R1 || m == W1)
            ack = start(2*a);

        if (ack) {
            auto q = (uint8_t*) p;
            if (m != R2) {
                for (auto i = 0; ack && i < n; ++i)
                    ack = wrByte(*q++);
            } else {
                ack = start(2*a + 1);
                for (auto i = 0; i < n; ++i)
                    *q++ = rdByte(i == n-1);
            }
        }

        if (m == R2 || m == W2 || !ack)
            stop();

        return ack;
    }

    bool start (uint8_t addr) const {
        sclLo();
        sclHi();
        sda = 0;
        return wrByte(addr);
    }

    void stop () const {
        sda = 0;
        sclHi();
        sda = 1;
        hold();
    }

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
        if (last)
            stop();
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

private:
    void hold () const {
        for (volatile int i = rate; --i >= 0; ) {}
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

// polled H/W version (see i2c::Gpio for bit-banged version)
template< uint32_t A >
struct Poll {
    using ID = uint8_t;
    enum { AE=1<<0, RL=1<<1, ST=1<<2, RD=1<<3 }; // used as flag bits in mode

    static constexpr IoReg<A> I2C {};
#if STM32F4
    enum { CR1=0x00,CR2=0x04,DR=0x10,SR1=0x14,SR2=0x18,
           CCR=0x1C,TRISE=0x20 };
#else
    enum { CR1=0x00,CR2=0x04,TIMINGR=0x10,TIMOUTR=0x14,
           ISR=0x18,ICR=0x1C,RXDR=0x24,TXDR=0x28 };
#endif

    struct Config {
        uint16_t ena;
        uint8_t mhz;
    };

    Pin sda, scl; // pin definitions must be kept in this order
    Config const cfg;

    Poll (uint16_t e, uint8_t f) : cfg { e, f } {}

    void init (char const* defs, uint32_t khz =400) {
        Pin::config(defs, &sda, 2);

        if (!sda) { // reset the I2C bus if SDA is stuck low
            scl.mode("OU");
            sda.mode("OU");
            for (auto i = 0; i < 32; ++i) {
                scl.toggle();
                cycles::usBusy(10);
            }
            scl = 1;
            assert(sda); // should now be unstuck
            Pin::config(defs, &sda, 2);
        }

        RCC(cfg.ena,1) = 1;
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
        RCC(cfg.ena, 1) = 0;
    }

    enum { R1=ST, R2=AE|ST|RD, W1=RL|ST, W2=AE }; // R1:04 R2:0D W1:06 W2:01

#if STM32F4
    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        auto waitFor = [](uint8_t bit) {
            while (!I2C[SR1](bit))
                if (I2C[SR1] & 0x4D00) { // TIMEOUT OVR AF BERR
                    I2C[SR1] = 0;
                    return false;
                }
            return true;
        };

        auto q = (uint8_t*) p;
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
                        I2C[DR] = *q++;
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
                    *q++ = I2C[DR];
                } while (--n > 0);
                break;
            default:
                fail();
        }
        if (m == W2)
            I2C[CR1](9) = 1; // STOP
        return true;
    }
#else
    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        startReq(a, m, n);

        auto q = (uint8_t*) p;
        while ((I2C[ISR] & 0x10F0) == 0) // ~TIMEOUT ~TCR ~TC ~STOPF ~NACKF
            if (I2C[ISR](2)) // RXNE
                *q++ = I2C[RXDR];
            else if (I2C[ISR](1)) // TXIS
                I2C[TXDR] = *q++;

        auto ok = !I2C[ISR](12) && !I2C[ISR](4); // ~TIMEOUT ~NACKF
        I2C[ICR] = I2C[ISR];
        return ok;
    }

protected:
    void startReq (uint8_t a, uint8_t m, uint8_t n) const {
        if (n == 0)
            m |= AE;
        I2C[CR2] = (((m&AE) != 0) << 25) // AUTOEND
                 | (((m&RL) != 0) << 24) // RELOAD
                 |             (n << 16) // NBYTES
                 | (((m&ST) != 0) << 13) // START
                 | (((m&RD) != 0) << 10) // RD_WRN
                 |             (a << 1); // SADD
    }
#endif

private:
    void setTiming (uint32_t khz) {
#if STM32F4
        if (khz < 10'000) {
            auto div = (1000 * cfg.mhz) / khz;
            I2C[CR2] = cfg.mhz;
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
            auto div = (1000 * cfg.mhz) / khz;
            auto presc = div/256;
            assert(presc < 16);
            div /= presc+1;
logf("11 %d %d %d+%d", cfg.mhz, presc, div/3, div-div/3);
            I2C[TIMINGR] = (presc<<28)
                           | (5<<20)
                           | (1<<16)
                           | (div/4<<8)
                           | (3*div/4<<0);
        } else // custom rate settings
            I2C[TIMINGR] = khz;

        // 25 ms timeout is approx 12x I2C clock in Mhz (i.e. sysclk/prescaler)
        // see table 394, p.1909 in RM0440 r8 for some suggested values
        // FIXME should this be cfg.mhz iso SystemCoreClock ?
        auto t = 12 * ((SystemCoreClock>>20) / ((khz>>28) + 1));
        assert(t < 4096);
        I2C[TIMOUTR] = (1<<15) | t; // TIMOUTEN
#endif
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Sync : Poll<A> {
    using BASE = Poll<A>;

    static constexpr IoReg<A> I2C {};

    struct Config : BASE::Config {
        Irq evIrq, erIrq, txIrq, rxIrq;
        DmaConfig<D,T,R> dma;
    };

    Config const cfg;

    Sync (Config const& c) : BASE (c.ena, c.mhz), cfg (c) {}

    void init (char const* defs, uint32_t khz =400) {
        BASE::init(defs, khz);
        I2C[BASE::CR1](14,2) = 0b11; // RXDMAEN TXDMAEN

        // peripheral address config and interrupt vector setup
        cfg.dma.init(A + BASE::TXDR, A + BASE::RXDR);

        SCB[0x10](4) = 1; // SEVONPEND
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma.idx,1) = 0; // may be shared

    // sync version, dma with wfe
    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        assert(n > 0);
        startReq(a, m, p, n);
        while (true) {
cycles::usBusy(15);
            if (!cfg.dma.isRunning())
                break;
            if (cfg.dma.completed() == 0)
                asm ("wfe");
            uint32_t isr = I2C[BASE::ISR];
            if ((isr & 0x10F0) != 0) // TIMEOUT TCR TC STOPF NACKF
                break;
        }
        Task::irqClear(cfg.evIrq);
        //Task::irqClear(cfg.erIrq);
        Task::irqClear(cfg.txIrq);
        Task::irqClear(cfg.rxIrq);
        return finishReq(m, p, n);
    }

protected:
    void startReq (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        // must set up DMA before START, see 33.4.16, p.1003 in RM0393 v2
        if (m != BASE::R2)
            cfg.dma.txStart(p, n);
        else
            cfg.dma.rxStart(p, n);

        BASE::startReq(a, m, n);
        I2C[BASE::CR1](4,4) = 0b1111; // ERRIE TCIE STOPIE NACKIE
    }

    bool finishReq (uint8_t m, void* p, uint8_t n) const {
        //cfg.dma.done();
cfg.dma.completed();
        I2C[BASE::CR1](4,4) = 0; // ~ERRIE ~TCIE ~STOPIE ~NACKIE
        if (m == BASE::R2)
            cache::inval(p, n);
        auto ok = !I2C[BASE::ISR](12) && ~I2C[BASE::ISR](4); // ~TIMEOUT ~NACKF
        I2C[BASE::ICR] = I2C[BASE::ISR];
        return ok;
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Async : Sync<A,D,T,R>, Task {
    using BASE = Sync<A,D,T,R>;
    using BASE::Sync, BASE::cfg;

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

    void interrupt () {
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

} // namespace jeeh
