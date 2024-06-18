namespace jeeh {

// polled H/W version (see I2cGpio for bit-banged version)
template< uint32_t A >
struct I2cHw {
    using ID = uint8_t;
    enum { AE = 1, RL = 2, ST = 4, RD = 8 }; // used as flag bits in Mode

    static constexpr IoReg<A> I2C {};
    enum {
        CR1=0x00,CR2=0x04,TIMINGR=0x10,ISR=0x18,ICR=0x1C,RXDR=0x24,TXDR=0x28
    };

    struct Config {
        uint16_t ena;
        uint8_t mhz;
    };

    Config const cfg;

    I2cHw (uint16_t e, uint8_t f) : cfg { e, f } {}

    void init (char const* defs, uint16_t speed) {
        Pin::config(defs);

        RCC(cfg.ena,1) = 1;
        assert(speed == 400 || speed == 1000); // TODO
        //I2C[TIMINGR] = 0x0070'2991; // magic! 400 kHz @ 80 MHz
        I2C[TIMINGR] = 0x0030'0F33; // magic! 1 MHz @ 80 MHz
        I2C[CR1](0) = 1; // PE
    }

    void deinit () {
        RCC(cfg.ena, 1) = 0;
    }

    enum { R1 = ST|RD, R2 = AE|ST|RD, W1 = RL|ST, W2 = AE };

    void transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        startReq(a, m, n);

        auto q = (uint8_t*) p;
        switch (m) {
            case R1:
                while (!I2C[ISR](6)) // ~TC
                    if (I2C[ISR](1)) // TXIS
                        I2C[TXDR] = *q++;
                break;
            case W1:
                while (!I2C[ISR](7)) // ~TCR
                    if (I2C[ISR](1)) // TXIS
                        I2C[TXDR] = *q++;
                break;
            case W2:
                while (!I2C[ISR](5)) // ~STOPF
                    if (I2C[ISR](1)) // TXIS
                        I2C[TXDR] = *q++;
                break;
            case R2:
                while (!I2C[ISR](5)) // ~STOPF
                    if (I2C[ISR](2)) // RXNE
                        *q++ = I2C[RXDR];
                break;
            default: fail();
        }
    }

    uint32_t read (uint8_t a, uint8_t r) const {
        uint32_t v = 0;
        read(a, r, &v, 1);
        return v;
    }

    void read (uint8_t a, uint8_t r, void* p, uint8_t n) const {
        transfer(a, R1, &r, 1);
        transfer(a, R2, p, n);
    }

    void write (uint8_t a, uint8_t r, uint8_t v) const {
        write(a, r, &v, 1);
    }

    void write (uint8_t a, uint8_t r, void const* p, uint8_t n) const {
        transfer(a, W1, &r, 1);
        transfer(a, W2, (void*) p, n);
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
struct I2cDma : I2cHw<A>, Device {
    using HW = I2cHw<A>;

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

    struct Config : HW::Config {
        Irq evIrq, erIrq;
        uint8_t dma, txReq, rxReq; // 0-based
    };

    Config const cfg;

    I2cDma (Config const& c) : HW (c.ena, c.mhz), Device ('I'), cfg (c) {}

    void init (char const* defs, int speed) {
        HW::init(defs, speed);
        I2C[HW::CR1](14,2) = 0b11; // RXDMAEN TXDMAEN

        RCC(ena::DMA1+cfg.dma, 1) = 1;
#if STM32L0 | STM32L4
        DMA[0xA8](4*T,4) = cfg.txReq; // CSELR
        DMA[0xA8](4*R,4) = cfg.rxReq; // CSELR
#endif
        DTX[CPAR] = A + HW::TXDR;
        DRX[CPAR] = A + HW::RXDR;
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

        irqInstall((uint8_t) cfg.evIrq);
        //irqInstall((uint8_t) cfg.erIrq);
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma, 1) = 0; // may be shared

    // sync versions, dma with wfe
    void transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        startReq(a, m, p, n);

        while (DTX[CCR](0) || DRX[CCR](0)) // EN
            asm ("wfe");

        if (m == HW::R2)
            cache::inval(p, n);
    }

    uint32_t read (uint8_t a, uint8_t r) const {
        uint32_t v = 0;
        read(a, r, &v, 1);
        return v;
    }

    void read (uint8_t a, uint8_t r, void* p, uint8_t n) const {
        transfer(a, HW::R1, &r, 1);
        transfer(a, HW::R2, (void*) p, n);

    }

    void write (uint8_t a, uint8_t r, uint8_t v) const {
        write(a, r, &v, 1);
    }

    void write (uint8_t a, uint8_t r, void const* p, uint8_t n) const {
        transfer(a, HW::W1, &r, 1);
        transfer(a, HW::W2, (void*) p, n);
    }

protected:
    void startReq (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        HW::startReq(a, m, n);
        I2C[HW::CR1](5,2) = 0b11; // TCIE STOPIE

        if (m != HW::R2) {
            cache::clean(p, n);
            DTX[CMAR] = (uintptr_t) p;
            DTX[CNDTR] = n;
            DTX[CCR](0) = 1; // EN
        } else {
            DRX[CMAR] = (uintptr_t) p;
            DRX[CNDTR] = n;
            DRX[CCR](0) = 1; // EN
        }
    }

private:
    Chain msgs;

    void startAsync (Message& m) {
        uint8_t mode = m.mLen >> 8, len = m.mLen;
        startReq(m.mTag, mode, m.mPtr, len);
    }

    // async version, started from a msg
    void start (Message& m) override {
        if (!msgs.append(m))
            startAsync(m);
    }
    void finish () override {
        auto mp = msgs.pull();
        if (mp != nullptr) {
            if (mp->mPtr != nullptr)
                cache::inval(mp->mPtr, mp->mLen);
            reply(mp);
        }
        mp = msgs.first();
        if (mp != nullptr)
            startAsync(*mp);
    }

    bool interrupt (int) override {
        I2C[HW::CR1](5,2) = 0; // TCIE STOPIE
        DTX[CCR](0) = 0; // ~EN
        DRX[CCR](0) = 0; // ~EN
        return !msgs.isEmpty();
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct I2cDev : I2cDma<A,D,T,R> {
    using I2cDma<A,D,T,R>::I2cDma;
    using HW = I2cHw<A>;

    void transfer (uint8_t a, uint8_t m, void* p, uint8_t n) const {
        uint16_t len = (m<<8) | n;
        Message msg { 'I', a, len, (uint8_t*) p };
        sys::call(msg); // async with thread suspend
    }

    uint32_t read (uint8_t a, uint8_t r) const {
        uint32_t v = 0;
        read(a, r, &v, 1);
        return v;
    }

    void read (uint8_t a, uint8_t r, void* p, uint8_t n) const {
        transfer(a, HW::R1, &r, 1);
        transfer(a, HW::R2, (void*) p, n);

    }

    void write (uint8_t a, uint8_t r, uint8_t v) const {
        write(a, r, &v, 1);
    }

    void write (uint8_t a, uint8_t r, void const* p, uint8_t n) const {
        transfer(a, HW::W1, &r, 1);
        transfer(a, HW::W2, (void*) p, n);
    }
};

} // namespace jeeh
