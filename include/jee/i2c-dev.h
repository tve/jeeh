namespace jeeh {

// polled H/W version (see I2cGpio for bit-banged version)
template< uint32_t A >
struct I2cHw {
    using ID = uint8_t;

    static constexpr IoReg<A> I2C {};
    enum { CR1=0x00,CR2=0x04,TIMINGR=0x10,ISR=0x18,RXDR=0x24,TXDR=0x28 };

    struct Config {
        uint16_t ena;
        uint8_t mhz;
    };

    Config const cfg;

    I2cHw (uint16_t e, uint8_t f) : cfg { e, f } {}

    void init (char const* defs, uint16_t speed) {
        Pin::config(defs);

        RCC(cfg.ena,1) = 1;
        assert(speed == 1000); // TODO
        //I2C[TIMINGR] = 0x0070'2991; // magic! 400 kHz @ 80 MHz
        I2C[TIMINGR] = 0x0030'0F33; // magic! 1 MHz @ 80 MHz
        I2C[CR1](0) = 1; // PE
    }

    void deinit () {
        RCC(cfg.ena, 1) = 0;
    }

    Dev<I2cHw> dev (uint8_t a) {
        return { a, *this };
    }

    uint32_t read (uint8_t a, uint32_t r) const {
        uint32_t v = 0;
        read(a, r, &v, 1);
        return v;
    }

    void read (uint8_t a, uint32_t r, void* p, uint8_t n) const {
        I2C[CR2] = (1<<16) | (1<<13) | (a<<1); // NBYTES START SADD
        I2C[TXDR] = r;
        while (I2C[ISR](6) == 0) {} // ~TC

        I2C[CR2] = // AUTOEND NBYTES START DIR SADD
                (1<<25) | (n<<16) | (1<<13) | (1<<10) | (a<<1);
        while (I2C[ISR](15) == 0) {} // ~BUSY

        auto q = (uint8_t*) p;
        while (I2C[ISR](15)) // BUSY
            if (I2C[ISR](2)) // RXNE
                *q++ = I2C[RXDR];
    }

    void write (uint8_t a, uint32_t r, uint16_t v) const {
        write(a, r, &v, 1);
    }

    void write (uint8_t a, uint32_t r, void const* p, uint8_t n) const {
        I2C[CR2] = // AUTOEND NBYTES START SADD
                (1<<25) | ((n+1)<<16) | (1<<13) | (a<<1);
        I2C[TXDR] = r;
        while (I2C[ISR](15) == 0) {} // ~BUSY

        auto q = (uint8_t const*) p;
        while (I2C[ISR](15)) // BUSY
            if (I2C[ISR](1)) // TXIS
                I2C[TXDR] = *q++;
    }

    void enable () const { fail(); }
    void disable () const { fail(); }
    int transfer (int) const { fail(); }
    void bufferIO (uint8_t*, uint16_t, bool) const { fail(); }
};

// DMA version, either sync-wfe or async (i.e. msgs sent to this device)
template< uint32_t A, uint32_t D, int T, int R >
struct I2cDma : I2cHw<A>, Device {
    using HW = I2cHw<A>;

#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STL32L4
    enum { ISR=0x00, IFCR=0x04,CCR=0x08,CNDTR=0x0C,CPAR=0x10,CMAR=0x14 };
    enum { CHAN_STEP=0x14 };
#else
    enum { ISR=0x00, IFCR=0x08,CCR=0x10,CNDTR=0x14,CPAR=0x18,CMAR=0x1C };
    enum { CHAN_STEP=0x18 };
#endif
#if STM32L0 | STM32L4
    enum { CSELR=0xA8 };
#endif

    static constexpr IoReg<D> DMA {};
    static constexpr IoReg<D+CHAN_STEP*T> DCT {}; // DMA channel TX
    static constexpr IoReg<D+CHAN_STEP*R> DCR {}; // DMA channel RX

    struct Config : I2cHw<A>::Config {
        Irq txIrq, rxIrq;
        uint8_t dma, txReq, rxReq; // 0-based
    };

    Config const cfg;

    I2cDma (Config const& c) : I2cHw<A> (c.ena, c.mhz), Device ('I'), cfg (c) {}

    void init (char const* defs, int speed) {
        I2cHw<A>::init(defs, speed);
        HW::I2C[HW::CR2](0,2) = 0b11; // RXDMAEN TXDMAEN

        RCC(ena::DMA1+cfg.dma, 1) = 1;
#if STM32L0 | STM32L4
        DMA[CSELR](4*T,4) = cfg.txReq;
        DMA[CSELR](4*R,4) = cfg.rxReq;
#endif
        DCT[CPAR] = A + HW::DR;
        DCR[CPAR] = A + HW::DR;
#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
        DCT[CCR] = 0b1001'0010; // MINC DIR TCIE
        DCR[CCR] = 0b1000'0010; // MINC TCIE
#elif STM32H7
        DCT[CCR] = 0b0100'0101'0000; // MINC DIR TCIE
        DCR[CCR] = 0b0100'0001'0000; // MINC TCIE
#else
        DCT[CCR] = (cfg.txReq<<25) | 0b0100'0101'0000; // CHSEL MINC DIR TCIE
        DCR[CCR] = (cfg.rxReq<<25) | 0b0100'0001'0000; // CHSEL MINC TCIE
#endif

        irqInstall((uint8_t) cfg.txIrq);
        irqInstall((uint8_t) cfg.rxIrq);
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma, 1) = 0; // may be shared

    // sync version, dma with wfe
    void bufferIO (uint8_t* buf, uint16_t len, bool send) const {
        startReq(send, buf, len);
        while (DCT[CCR](0) != 0 || DCR[CCR](0) != 0) // EN
            asm ("wfe");
        I2cHw<A>::disable();
        if (!send)
            cache::inval(buf, len);
    }

private:
    Chain msgs;

    void startReq (bool send, uint8_t* buf, uint16_t len) const {
        I2cHw<A>::enable();
        if (!send) {
            DCR[CMAR] = (uint32_t) buf;
            DCR[CNDTR] = len;
            DCR[CCR](0) = 1; // EN
        }
        // always send (RXIDLE mode is troublesome w/ DMA)
        cache::clean(buf, len);
        DCT[CMAR] = (uint32_t) buf;
        DCT[CNDTR] = len;
        DCT[CCR](0) = 1; // EN
    }

    // async version, started from a msg
    void start (Message& m) override {
        if (!msgs.append(m))
            startReq(m.mTag == 'W', m.mPtr, m.mLen);
    }

    void finish () override {
        auto mp = msgs.pull();
        if (mp != nullptr) {
            I2cHw<A>::disable();
            if (mp->mPtr != nullptr)
                cache::inval(mp->mPtr, mp->mLen);
            reply(mp);
        }
        mp = msgs.first();
        if (mp != nullptr)
            startReq(mp->mTag == 'W', mp->mPtr, mp->mLen);
    }

    bool interrupt (int) override {
#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
        if (DMA[ISR](4*T)) { // GIF
            DCT[CCR](0) = 0; // ~EN
            DMA[IFCR] = 1<<(4*T);
        } else if (DMA[ISR](4*R)) { // GIF
            DCR[CCR](0) = 0; // ~EN
            DMA[IFCR] = 1<<(4*R);
        } else
            fail();
#else
        constexpr uint8_t ifcBits [] = { 0, 6, 16, 22 };
        if (DMA[T&~3](5+ifcBits[T&3])) // tx TCIF
            DMA[IFCR+(T&~3)] = 0b111101 << ifcBits[T&3]; // clr irq
        else if (DMA[R&~3](5+ifcBits[R&3])) // rx TCIF
            DMA[IFCR+(R&~3)] = 0b111101 << ifcBits[R&3]; // clr irq
        else
            fail();
#endif
        if (DCT[CCR](0) || DCR[CCR](0)) // EN
            return false; // still in progress

        // clear OVR flag, in case the data was never read
        (void) +HW::I2C.byte(HW::DR);
        (void) +HW::I2C[HW::SR];
        return !msgs.isEmpty();
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct I2cDev : I2cDma<A,D,T,R> {
    using I2cDma<A,D,T,R>::I2cDma;

    void bufferIO (uint8_t* buf, uint16_t len, bool send) const {
        Message m { 'I', send ? 'W' : 'R', len, buf };
        sys::call(m); // async with thread suspend
    }
};

} // namespace jeeh
