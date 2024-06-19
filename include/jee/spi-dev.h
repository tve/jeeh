namespace jeeh {

// polled H/W version (see SpiGpio for bit-banged version)
template< uint32_t A >
struct SpiPoll {
    using ID = Pin;

    static constexpr IoReg<A> SPI {};
    enum { CR1=0x00, CR2=0x04, SR=0x08, DR=0x0C }; // SPI regs

    struct Config {
        uint16_t ena;
        uint8_t mhz;
    };

    Config const cfg;
    Pin nsel;

    SpiPoll (uint16_t e, uint8_t f) : cfg { e, f } {}

    void init (char const* defs, int khz) {
        Pin pins [4]; // mosi, miso, nclk, nsel
        Pin::config(defs, pins, sizeof pins);
        nsel = pins[3];
        disable(); // start with NSEL high

        auto div = 0; // determine clock divider
        while ((1000*cfg.mhz >> (div+1)) > khz)
            ++div;
div = 3;

        RCC(cfg.ena, 1) = 1;
        SPI[CR1] = (div<<3) | (1<<2); // BD MSTR
#if STM32F1 | STM32L0 | STM32L4
        SPI[CR2] = (1<<2); // SSOE
#else
        SPI[CR2] = (1<<12) | (7<<8) | (1<<2); // FRXTH DS SSOE
#endif
        SPI[CR1](6) = 1; // SPE
    }

    void deinit () {
        RCC(cfg.ena, 1) = 0;
    }

    void enable () const {
        for (volatile int i = 10; --i >= 0; ) {}
        nsel = 0;
        for (volatile int i = 10; --i >= 0; ) {}
    }

    void disable () const {
        for (volatile int i = 10; --i >= 0; ) {}
        nsel = 1;
        for (volatile int i = 10; --i >= 0; ) {}
    }

    int transfer (int v) const {
        SPI.byte(DR) = v;
        while (!SPI[SR](0)) {} // ~RXNE
        return SPI.byte(DR);
    }

    enum { R1, W1, R2, W2 };

    uint8_t transfer (uint8_t m, uint8_t* p, uint16_t n) const {
        uint8_t r = 0;
        if (m <= W1)
            enable();

        if (n > 0) {
            auto q = (uint8_t*) p;
            if (m != R2) {
                SPI.byte(DR) = *q++;
                while (--n != 0) {
                    while (!SPI[SR](1)) {} // ~TXE
                    SPI.byte(DR) = *q++;
                    while (!SPI[SR](0)) {} // ~RXNE
                    (void) +SPI.byte(DR);
                }
                while (!SPI[SR](0)) {} // ~RXNE
                r = SPI.byte(DR);
            } else {
                SPI.byte(DR) = 0;
                while (--n != 0) {
                    while (SPI[SR](1)) {} // ~TXE
                    SPI.byte(DR) = 0;
                    while (SPI[SR](0)) {} // ~RXNE
                    *q++ = SPI.byte(DR);
                }
                while (SPI[SR](0)) {} // ~RXNE
                *q = SPI.byte(DR);
            }
        }

        if (m >= R2)
            disable();
        return r;
    }

    void bufferIO (uint8_t* buf, uint16_t len, bool send) const {
fail();
    }
};

// DMA version, either sync-wfe or async (i.e. msgs sent to this device)
template< uint32_t A, uint32_t D, int T, int R >
struct SpiSync : SpiPoll<A>, Device {
    using HW = SpiPoll<A>;

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
    static constexpr IoReg<D+CHAN_STEP*T> DTX {}; // DMA channel TX
    static constexpr IoReg<D+CHAN_STEP*R> DRX {}; // DMA channel RX

    struct Config : SpiPoll<A>::Config {
        Irq txIrq, rxIrq;
        uint8_t dma, txReq, rxReq; // 0-based
    };

    Config const cfg;

    SpiSync (Config const& c) : SpiPoll<A> (c.ena, c.mhz), Device ('S'), cfg (c) {}

    void init (char const* defs, int speed) {
        SpiPoll<A>::init(defs, speed);
        HW::SPI[HW::CR2](0,2) = 0b11; // RXDMAEN TXDMAEN

        RCC(ena::DMA1+cfg.dma, 1) = 1;
#if STM32L0 | STM32L4
        DMA[CSELR](4*T,4) = cfg.txReq;
        DMA[CSELR](4*R,4) = cfg.rxReq;
#endif
        DTX[CPAR] = A + HW::DR;
        DRX[CPAR] = A + HW::DR;
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

        irqInstall((uint8_t) cfg.txIrq);
        irqInstall((uint8_t) cfg.rxIrq);
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma, 1) = 0; // may be shared

    // sync version, dma with wfe
    void bufferIO (uint8_t* buf, uint16_t len, bool send) const {
        startReq(send, buf, len);
        while (DTX[CCR](0) || DRX[CCR](0)) // EN
            asm ("wfe");
        SpiPoll<A>::disable();
        if (!send)
            cache::inval(buf, len);
    }

private:
    Chain msgs;

    void startReq (bool send, uint8_t* buf, uint16_t len) const {
        SpiPoll<A>::enable();
        if (!send) {
            DRX[CMAR] = (uint32_t) buf;
            DRX[CNDTR] = len;
            DRX[CCR](0) = 1; // EN
        }
        // always send (RXIDLE mode is troublesome w/ DMA)
        cache::clean(buf, len);
        DTX[CMAR] = (uint32_t) buf;
        DTX[CNDTR] = len;
        DTX[CCR](0) = 1; // EN
    }

    // async version, started from a msg
    void start (Message& m) override {
        if (!msgs.append(m))
            startReq(m.mTag == 'W', m.mPtr, m.mLen);
    }

    void finish () override {
        auto mp = msgs.pull();
        if (mp != nullptr) {
            SpiPoll<A>::disable();
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
            DTX[CCR](0) = 0; // ~EN
            DMA[IFCR] = 1<<(4*T);
        } else if (DMA[ISR](4*R)) { // GIF
            DRX[CCR](0) = 0; // ~EN
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
        if (DTX[CCR](0) || DRX[CCR](0)) // EN
            return false; // still in progress

        // clear OVR flag, in case the data was never read
        (void) +HW::SPI.byte(HW::DR);
        (void) +HW::SPI[HW::SR];
        return !msgs.isEmpty();
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct SpiAsync : SpiSync<A,D,T,R> {
    using SpiSync<A,D,T,R>::SpiSync;

    void bufferIO (uint8_t* buf, uint16_t len, bool send) const {
        Message m { 'S', send ? 'W' : 'R', len, buf };
        sys::call(m); // async with thread suspend
    }
};

} // namespace jeeh
