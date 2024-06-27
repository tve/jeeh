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

    Config const cfg;

    Poll (uint16_t e, uint8_t f) : cfg { e, f } {}

    void init (char const* defs, uint32_t timing) {
        Pin::config(defs);

        RCC(cfg.ena,1) = 1;
        I2C[TIMINGR] = timing;

#if 0
        // 25 ms timeout is approx 12x I2C clock in Mhz (i.e. sysclk/prescaler)
        // see table 394, p.1909 in RM0440 r8 for some suggested values
        auto t = 12 * ((SystemCoreClock/1'000'000) / ((timing>>28) + 1));
        assert(t < 4096);
        I2C[TIMOUTR] = (1<<15) | t; // TIMOUTEN
#endif

        I2C[CR1](0) = 1; // PE
    }

    void deinit () {
        RCC(cfg.ena, 1) = 0;
    }

    enum { R1 = ST, R2 = AE|ST|RD, W1 = RL|ST, W2 = AE };

    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        startReq(a, m, n);

        auto q = (uint8_t*) p;
        if (m != R2)
            while ((I2C[ISR] & (0b111<<5)) == 0) { // ~TC or ~TCR or ~STOPF
                if (I2C[ISR](4)) { // NACKF
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
    using BASE::Poll; // constructor

#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
    enum { ISR=0x00, IFCR=0x04,CCR=0x08,CNDTR=0x0C,CPAR=0x10,CMAR=0x14 };
    enum { CHAN_STEP=0x14 };
#else
    enum { ISR=0x00, IFCR=0x08,CCR=0x10,CNDTR=0x14,CPAR=0x18,CMAR=0x1C };
    enum { CHAN_STEP=0x18 };
#endif

    static constexpr IoReg<A>             I2C {};
    static constexpr IoReg<D>             DMA {};
    static constexpr IoReg<D+CHAN_STEP*T> DTX {}; // DMA channel TX
    static constexpr IoReg<D+CHAN_STEP*R> DRX {}; // DMA channel RX

    struct Config : BASE::Config {
        Irq evIrq, erIrq;
        uint8_t dma, txReq, rxReq; // 0-based
    };

    Config const cfg;

    Sync (Config const& c) : BASE (c.ena, c.mhz), Device ('I'), cfg (c) {}

    void init (char const* defs, uint32_t timing) {
        BASE::init(defs, timing);
        I2C[BASE::CR1](14,2) = 0b11; // RXDMAEN TXDMAEN

        initDma();

        // peripheral address config and interrupt vector setup
        DTX[CPAR] = A + BASE::TXDR;
        DRX[CPAR] = A + BASE::RXDR;

        irqInstall((uint8_t) cfg.evIrq);
        //irqInstall((uint8_t) cfg.erIrq);
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma, 1) = 0; // may be shared

    // sync versions, dma with wfe
    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
//if (n == 0) return 0;

        startReq(a, m, p, n);
        while (DTX[CCR](0) || DRX[CCR](0)) // EN
        //while (I2C[BASE::CR1](5,2)) // TCIE STOPIE
            asm ("wfe");
        if (m == BASE::R2)
            cache::inval(p, n);
        return true; // TODO
    }

protected:
    void startReq (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        // must set op DMA before START, see 33.4.16, p.1003 in RM0393 v2
        // (although it seems to work just as well the other way around?)

        if (m != BASE::R2) {
            cache::clean(p, n);

            DTX[CMAR] = (uintptr_t) p;
            DTX[CNDTR] = n;
            DTX[CCR](0) = 1; // EN
        } else {
            DRX[CMAR] = (uintptr_t) p;
            DRX[CNDTR] = n;
            DRX[CCR](0) = 1; // EN
        }

        BASE::startReq(a, m, n);
        I2C[BASE::CR1](5,2) = 0b11; // TCIE STOPIE
assert(DTX[CCR](0) || DRX[CCR](0));
    }

private:
    Chain msgs;

    // TODO this is the same code in I2C and SPI
    void initDma () const {
        RCC(ena::DMA1+cfg.dma,1) = 1;

        // channel/stream/request setup (confusing naming differences!)
#if STM32G4
        RCC(ena::DMAMUX, 1) = 1;
#elif STM32H7
#define DMAMUX DMAMUX1
#endif
#if STM32G4 | STM32H7
        DMAMUX[32*cfg.dma+4*T] = cfg.txReq;
        DMAMUX[32*cfg.dma+4*R] = cfg.rxReq;
#elif STM32L0 | STM32L4
        DMA[0xA8](4*T,4) = cfg.txReq; // CSELR
        DMA[0xA8](4*R,4) = cfg.rxReq; // CSELR
#endif

        // channel configuration
#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
        DTX[CCR] = 0b1001'0010; // MINC DIR TCIE
        DRX[CCR] = 0b1000'0010; // MINC TCIE
#elif STM32H7
        DTX[CCR] = 0b0100'0101'0000; // MINC DIR TCIE
        DRX[CCR] = 0b0100'0001'0000; // MINC TCIE
#else
        DTX[CCR] = (cfg.txReq<<25) | 0b0100'0101'0000; // CHSEL MINC DIR TCIE
        DRX[CCR] = (cfg.rxReq<<25) | 0b0100'0001'0000; // CHSEL MINC TCIE
#endif
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
        if ((mp->mLen >> 8) == BASE::R2)
            cache::inval(mp->mPtr, (uint8_t) mp->mLen);
        reply(mp);
        if (!msgs.isEmpty())
            startAsync(*msgs.first());
    }

    void startAsync (Message& m) {
        startReq(m.mTag, m.mLen >> 8, m.mPtr, (uint8_t) m.mLen);
    }

    bool interrupt (int) override {
{ Pin("B8") = 1; }
        I2C[BASE::CR1](5,2) = 0; // TCIE STOPIE
        DTX[CCR](0) = 0; // ~EN
        DRX[CCR](0) = 0; // ~EN
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
        return true; // TODO
    }
};

} // namespace jeeh
