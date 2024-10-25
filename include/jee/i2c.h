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

    // one byte address, to/from buffer
    bool read (uint8_t r, void* p, uint8_t n) const {
        return transfer(bus.R1, &r, 1)
            && transfer(bus.R2, p, n);
    }
    bool write (uint8_t r, void const* p, uint8_t n) const {
        return transfer(bus.W1, &r, 1)
            && transfer(bus.W2, (void*) p, n);
    }

    // two byte address, to/from buffer
    bool read16 (uint16_t r, void* p, uint8_t n) const {
        return transfer(bus.R1, &r, 2)
            && transfer(bus.R2, p, n);
    }
    bool write16 (uint16_t r, void const* p, uint8_t n) const {
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

    void init (char const* desc, uint16_t khz =400) {
        Pin::config(desc, &sda, 2);
        Pin::config(":OU,", &sda, 2);

        scl = 1;
        sda = 1;

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
    enum { CR1=0x00,CR2=0x04,TIMINGR=0x10,TIMOUTR=0x14,
           ISR=0x18,ICR=0x1C,RXDR=0x24,TXDR=0x28 };

    struct Config {
        uint16_t ena;
        uint8_t mhz;
    };

    Pin sda, scl; // pin definitions must be kept in this order
    Config const cfg;

    Poll (uint16_t e, uint8_t f) : cfg { e, f } {}

    void init (char const* defs, uint32_t timing) {
        Pin::config(defs, &sda, 2);

        RCC(cfg.ena,1) = 1;
        I2C[TIMINGR] = timing;

        // 25 ms timeout is approx 12x I2C clock in Mhz (i.e. sysclk/prescaler)
        // see table 394, p.1909 in RM0440 r8 for some suggested values
        auto t = 12 * ((SystemCoreClock>>20) / ((timing>>28) + 1));
        assert(t < 4096);
        I2C[TIMOUTR] = (1<<15) | t; // TIMOUTEN

        I2C[CR1](0) = 1; // PE
    }

    void deinit () {
        Pin::config(":F,", &sda, 2);
        I2C[CR1](0) = 0; // ~PE
        RCC(cfg.ena, 1) = 0;
    }

    enum { R1=ST, R2=AE|ST|RD, W1=RL|ST, W2=AE }; // R1:04 R2:0D W1:06 W2:01

    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        startReq(a, m, n);

        auto q = (uint8_t*) p;
        if (m != R2)
            while ((I2C[ISR] & (0b111<<5)) == 0) { // ~TCR ~TC ~STOPF
                if (I2C[ISR](12) || I2C[ISR](4)) { // TIMEOUT NACKF
                    I2C[ICR] = I2C[ISR];
                    return false;
                }
                if (!I2C[ISR](15) || I2C[ISR](5)) // ~BUSY or STOPF
                    break;
                if (I2C[ISR](1)) // TXIS
                    I2C[TXDR] = *q++;
            }
        else // R2
            while (!I2C[ISR](5)) // ~STOPF
                if (I2C[ISR](2)) // RXNE
                    *q++ = I2C[RXDR];

if (m == W2) cycles::usBusy(1);
        return true;
    }

protected:
    void startReq (uint8_t a, uint8_t m, uint8_t n) const {
        I2C[ICR] = (1<<5); // STOPCF
        if (n == 0)
            m |= AE;
        I2C[CR2] = (((m&AE) != 0) << 25) // AUTOEND
                 | (((m&RL) != 0) << 24) // RELOAD
                 |             (n << 16) // NBYTES
                 | (((m&ST) != 0) << 13) // START
                 | (((m&RD) != 0) << 10) // RD_WRN
                 |             (a << 1); // SADD
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Sync : Poll<A> {
    using BASE = Poll<A>;

    static constexpr IoReg<A> I2C {};

    struct Config : BASE::Config {
        Irq evIrq, erIrq;
    };

    Config const cfg;
    DmaConfig<D,T,R> const dma;

    Sync (Config const& c, DmaConfig<D,T,R> d)
        : BASE (c.ena, c.mhz), cfg (c), dma (d) {}

    void init (char const* defs, uint32_t timing) {
        BASE::init(defs, timing);
        I2C[BASE::CR1](14,2) = 0b11; // RXDMAEN TXDMAEN

        // peripheral address config and interrupt vector setup
        dma.init(A + BASE::TXDR, A + BASE::RXDR);

        SCB[0x10](4) = 1; // SEVONPEND
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma, 1) = 0; // may be shared

    // sync version, dma with wfe
    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        assert(n > 0);
        startReq(a, m, p, n);
        while (true) {
            BlockIRQ irq;
            if (!dma.isRunning())
                break;
            uint32_t isr = I2C[BASE::ISR];
            if (m != BASE::R2) {
                // TIMEOUT TCR TC NACKF
                if ((isr & (1<<12)) || (isr & (0b1101<<4))) {
                    //I2C[BASE::ICR] = I2C[BASE::ISR];
                    break; // TODO assume finish will return false?
                }
            }
            if (!(isr & (1<<15)) || (isr & (1<<5))) // ~BUSY or STOPF
                break;
            auto dc = dma.completed();
            if (dc == 0)
                asm ("wfe");
        }
        Worker::irqClear(cfg.evIrq);
        //Worker::irqClear(cfg.erIrq);
        return finishReq(m, p, n);
    }

protected:
    void startReq (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        // must set up DMA before START, see 33.4.16, p.1003 in RM0393 v2
        if (m != BASE::R2)
            dma.txStart(p, n);
        else
            dma.rxStart(p, n);

        BASE::startReq(a, m, n);
        I2C[BASE::CR1](4,3) = 0b111; // TCIE STOPIE NACKIE
    }

    bool finishReq (uint8_t m, void* p, uint8_t n) const {
        I2C[BASE::CR1](4,3) = 0; // ~TCIE ~STOPIE ~NACKIE
        //dma.done();
        if (m == BASE::R2)
            cache::inval(p, n);
        bool nak = I2C[BASE::ISR](4); // NACKF
        I2C[BASE::ICR] = I2C[BASE::ISR];
        return !nak;
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Work : Sync<A,D,T,R>, Worker {
    using BASE = Sync<A,D,T,R>;
    using BASE::Sync, BASE::cfg, BASE::dma;

    enum TAG { RXDONE, TXDONE };

    Event pending;

    uint8_t init (char const* defs, int khz) {
        BASE::init(defs, khz);
        irqEnable(cfg.evIrq);
        //irqEnable(cfg.erIrq);
        return Worker::init();
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
    Event process (Event in, Event out, void*) override {
        switch (in.eTag) {
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
