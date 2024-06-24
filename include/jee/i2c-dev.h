// see examples/i2c/fram.cpp

namespace jeeh {

// polled H/W version (see I2cGpio for bit-banged version)
template< uint32_t A >
struct I2cPoll {
    using ID = uint8_t;
    enum { AE = 1, RL = 2, ST = 4, RD = 8 }; // used as flag bits in Mode

    static constexpr IoReg<A> I2C {};
    enum { CR1=0x00,CR2=0x04,TIMINGR=0x10,
           ISR=0x18,ICR=0x1C,RXDR=0x24,TXDR=0x28 };

    struct Config {
        uint16_t ena;
        uint8_t mhz;
    };

    Config const cfg;

    I2cPoll (uint16_t e, uint8_t f) : cfg { e, f } {}

    void init (char const* defs, uint16_t khz =400) {
        Pin::config(defs);

        RCC(cfg.ena,1) = 1;
        switch (khz) {
#if STM32G4 // TODO magic! G431 @ 170 MHz
            case 100:  I2C[TIMINGR] = 0x30A0'A7FB; break;
            case 400:  I2C[TIMINGR] = 0x1080'2D9B; break;
            case 1000: I2C[TIMINGR] = 0x0080'2172; break;
#endif
#if STM32L4 // TODO magic! L432 @ 80 MHz
            case 100:  I2C[TIMINGR] = 0x1090'9CEC; break;
            case 400:  I2C[TIMINGR] = 0x0070'2991; break;
            case 1000: I2C[TIMINGR] = 0x0030'0F33; break;
#endif
            default:   fail();
        }
        I2C[CR1](0) = 1; // PE
    }

    void deinit () {
        RCC(cfg.ena, 1) = 0;
    }

    enum { R1 = ST, R2 = AE|ST|RD, W1 = RL|ST, W2 = AE };

    bool transfer (ID a, uint8_t m, void* p, uint8_t n) const {
        startReq(a, m, n);

        auto q = (uint8_t*) p;
        if (m == R1) {
            while (!I2C[ISR](6)) // ~TC
                if (I2C[ISR](1)) // TXIS
                    I2C[TXDR] = *q++;
        } else if (m == R2) {
            while (!I2C[ISR](5)) // ~STOPF
                if (I2C[ISR](2)) // RXNE
                    *q++ = I2C[RXDR];
        } else if (m == W1) {
            while (!I2C[ISR](7)) // ~TCR
                if (I2C[ISR](1)) // TXIS
                    I2C[TXDR] = *q++;
        } else if (m == W2) {
            while (!I2C[ISR](5)) // ~STOPF
                if (I2C[ISR](1)) // TXIS
                    I2C[TXDR] = *q++;
        } else
            fail();

        return true; // TODO
    }

protected:
    void startReq (uint8_t a, uint8_t m, uint8_t n) const {
        I2C[ICR] = (1<<5); // STOPCF
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
struct I2cSync : I2cPoll<A>, Device {
    using BASE = I2cPoll<A>;
    using BASE::I2cPoll; // constructor

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

    I2cSync (Config const& c) : BASE (c.ena, c.mhz), Device ('I'), cfg (c) {}

    void init (char const* defs, int khz =400) {
        BASE::init(defs, khz);
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
        startReq(a, m, p, n);
        while (DTX[CCR](0) || DRX[CCR](0)) // EN
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
    }

private:
    Chain msgs;

    // TODO this is the same code in I2C and SPI
    void initDma () const {
        RCC(ena::DMA1+cfg.dma, 1) = 1;

        // channel/stream/request setup (confusing naming differences!)
#if STM32G4
        RCC(ena::DMAMUX, 1) = 1;
#elif STM32H7
#define DMAMUX DMAMUX1
#endif
#if STM32G4 | STM32H7
        DMAMUX[32*cfg.dma+4*T] = cfg.rxReq;
        DMAMUX[32*cfg.dma+4*R] = cfg.txReq;
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
        if (mp->mPtr != nullptr)
            cache::inval(mp->mPtr, mp->mLen);
        reply(mp);
        if (!msgs.isEmpty())
            startAsync(*msgs.first());
    }

    void startAsync (Message& m) {
        uint8_t mode = m.mLen >> 8, len = m.mLen;
        startReq(m.mTag, mode, m.mPtr, len);
    }

    bool interrupt (int) override {
        I2C[BASE::CR1](5,2) = 0; // TCIE STOPIE
        DTX[CCR](0) = 0; // ~EN
        DRX[CCR](0) = 0; // ~EN
        return !msgs.isEmpty();
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct I2cCall : I2cSync<A,D,T,R> {
    using BASE = I2cSync<A,D,T,R>;
    using BASE::I2cSync; // constructor

    bool transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        uint16_t len = (m<<8) | n;
        Message msg { BASE::dId, a, len, (uint8_t*) p };
        sys::call(msg); // async with thread suspend
        return true; // TODO
    }
};

} // namespace jeeh
