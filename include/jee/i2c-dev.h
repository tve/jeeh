// see examples/i2c/fram.cpp

namespace jeeh::i2c {

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

    Pin sda {}, scl {}; // pin definitions must be kept in this order
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
        Pin::config(":U,", &sda, 2); // keep SDA and SCL pulled-up
        RCC(cfg.ena, 1) = 0;
    }

    enum { R1 = ST, R2 = AE|ST|RD, W1 = RL|ST, W2 = AE };

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

// DMA version, either sync-wfe or async (i.e. msgs sent to this device)
template< uint32_t A, uint32_t D, int T, int R >
struct Sync : Poll<A>, Device {
    using BASE = Poll<A>;

    static constexpr IoReg<A> I2C {};

    struct Config : BASE::Config {
        Irq evIrq, erIrq;
        uint8_t Xdma, XtxReq, XrxReq; // 0-based
    };

    DmaConfig<D,T,R> const dma;
    Config const cfg;

    Sync (Config const& c)
        : BASE (c.ena, c.mhz), Device ('I'),
          dma { c.Xdma, c.XtxReq, c.XrxReq }, cfg (c) {}

    void init (char const* defs, uint32_t timing) {
        BASE::init(defs, timing);
        I2C[BASE::CR1](14,2) = 0b11; // RXDMAEN TXDMAEN

        // peripheral address config and interrupt vector setup
        dma.init(A + BASE::TXDR, A + BASE::RXDR);

        irqInstall((uint8_t) cfg.evIrq);
        //irqInstall((uint8_t) cfg.erIrq);
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma, 1) = 0; // may be shared

    // sync version, dma with wfe
    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        startReq(a, m, p, n);
        while (I2C[BASE::ISR](15) && // BUSY
                I2C[BASE::CR1](4,3)) // TCIE STOPIE NACKIE
            asm ("wfe");
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
        dma.done();
        if (m == BASE::R2)
            cache::inval(p, n);
        if (I2C[BASE::ISR](4)) { // NACKF
            I2C[BASE::ICR] = I2C[BASE::ISR];
            return false;
        }
        return true;
    }

private:
    Chain msgs;

    // async version, started from a msg
    void start (Message& m) override {
        if (!msgs.append(m))
            startAsync(m);
    }

    void finish () override {
        auto mp = msgs.pull();
        if (mp == nullptr)
            return;
        mp->mLen = finishReq(mp->mLen >> 8, mp->mPtr, (uint8_t) mp->mLen);
        reply(mp);
        if (!msgs.isEmpty())
            startAsync(*msgs.first());
    }

    void startAsync (Message& m) {
        uint8_t mode = m.mLen >> 8, len = m.mLen;
        startReq(m.mTag, mode, m.mPtr, len);
        if (mode == BASE::W1 && len == 0)
            finish(); // this may be recursive
    }

    bool interrupt (int) override {
        I2C[BASE::CR1](4,3) = 0; // ~TCIE ~STOPIE ~NACKIE
        return !msgs.isEmpty();
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Call : Sync<A,D,T,R> {
    using BASE = Sync<A,D,T,R>;
    using BASE::Sync; // constructor

    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        uint16_t len = (m<<8) | n;
        Message msg { BASE::dId, a, len, (uint8_t*) p };
        sys::call(msg); // async with thread suspend
        return msg.mLen;
    }
};

} // namespace jeeh
