namespace jeeh {

// polled H/W version (see I2cGpio for bit-banged version)
template< uint32_t A >
struct I2cHw {
    using ID = uint8_t;

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

    uint32_t read (uint8_t a, uint8_t r) const {
        uint32_t v = 0;
        read(a, r, &v, 1);
        return v;
    }

    void read (uint8_t a, uint8_t r, void* p, uint8_t n) const {
        startRead(a, r, n);

        auto q = (uint8_t*) p;
        while (I2C[ISR](15)) // BUSY
            if (I2C[ISR](2)) // RXNE
                *q++ = I2C[RXDR];
    }

    void write (uint8_t a, uint8_t r, uint8_t v) const {
        write(a, r, &v, 1);
    }

    void write (uint8_t a, uint8_t r, void const* p, uint8_t n) const {
        startWrite(a, r, n);

        auto q = (uint8_t const*) p;
        while (I2C[ISR](15)) // BUSY
            if (I2C[ISR](1)) // TXIS
                I2C[TXDR] = *q++;
    }

protected:
    void startRead (uint8_t a, uint8_t r, uint8_t n) const {
        I2C[CR2] = // NBYTES START SADD
                (1<<16) | (1<<13) | (a<<1);
        I2C[TXDR] = r;
        while (!I2C[ISR](6)) {} // ~TC

        I2C[CR2] = // AUTOEND NBYTES START DIR SADD
                (1<<25) | (n<<16) | (1<<13) | (1<<10) | (a<<1);
        while (!I2C[ISR](15)) {} // ~BUSY
    }

    void startWrite (uint8_t a, uint8_t r, uint8_t n) const {
        I2C[CR2] = // AUTOEND NBYTES START SADD
                (1<<25) | ((n+1)<<16) | (1<<13) | (a<<1);
        I2C[TXDR] = r;
        while (!I2C[ISR](15)) {} // ~BUSY, takes 8 cycles on L432
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
        I2C[HW::CR1](5) = 1; // STOPIE

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
    using HW::read;
    using HW::write;

    void read (uint8_t a, uint8_t r, void* p, uint8_t n) const {
        HW::startRead(a, r, n);

        DRX[CMAR] = (uintptr_t) p;
        DRX[CNDTR] = n;
        DRX[CCR](0) = 1; // EN

        while (DRX[CCR](0)) // EN
            asm ("wfe");
        cache::inval(p, n);
    }

    void write (uint8_t a, uint8_t r, void const* p, uint8_t n) const {
        HW::startWrite(a, r, n);

        cache::clean(p, n);
        DTX[CMAR] = (uintptr_t) p;
        DTX[CNDTR] = n;
        DTX[CCR](0) = 1; // EN

        while (DTX[CCR](0)) // EN
            asm ("wfe");
    }

private:
    Chain msgs;

    // async version, started from a msg
    void start (Message&) override { fail(); }
    void finish () override { fail(); }

    bool interrupt (int) override {
        if (I2C[HW::ISR](5)) { // STOPF
            I2C[HW::ICR] = (1<<5); // STOPCF
            DTX[CCR](0) = 0; // ~EN
            DRX[CCR](0) = 0; // ~EN
        }
        return !msgs.isEmpty();
    }
};

#if 0 // TODO
template< uint32_t A, uint32_t D, int T, int R >
struct I2cDev : I2cDma<A,D,T,R> {
    using I2cDma<A,D,T,R>::I2cDma;

    void bufferIO (uint8_t* buf, uint16_t len, bool send) const {
        Message m { 'I', send ? 'W' : 'R', len, buf };
        sys::call(m); // async with thread suspend
    }
};
#endif

} // namespace jeeh
