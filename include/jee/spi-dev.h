// see examples/spi/{endure,spif}.cpp

namespace jeeh::spi {

// polled H/W version (see spi::Gpio for bit-banged version)
template< uint32_t A >
struct Poll {
    using ID = Pin;

    static constexpr IoReg<A> SPI {};
    enum { CR1=0x00, CR2=0x04, SR=0x08, DR=0x0C }; // SPI regs

    struct Config {
        uint16_t ena;
        uint8_t mhz;
    };

    Config const cfg;
    Pin nsel;

    Poll (uint16_t e, uint8_t f) : cfg { e, f } {}

    void init (char const* defs, int khz) {
        Pin pins [4]; // mosi, miso, nclk, nsel
        Pin::config(defs, pins, sizeof pins);
        nsel = pins[3];
        disable(); // start with NSEL high

        auto div = 0; // determine clock divider
        while ((1000*cfg.mhz >> (div+1)) > khz)
            ++div;

        RCC(cfg.ena, 1) = 1;
        SPI[CR1] = (div<<3) | (1<<2); // BD MSTR
#if STM32F1 | STM32L0
        SPI[CR2] = (1<<2); // SSOE
#else
        SPI[CR2] = (1<<12) | (7<<8) | (1<<2); // FRXTH DS SSOE
#endif
        SPI[CR1](6) = 1; // SPE
    }

    void deinit () { RCC(cfg.ena, 1) = 0; }

    void enable () const { nsel.clear(); }
    void disable () const { nsel.set(); }

    int rwByte (int v) const {
        SPI.byte(DR) = v;
        while (!SPI[SR](0)) {} // ~RXNE
        return SPI.byte(DR);
    }

    uint8_t transfer (uint8_t w, uint8_t* p, uint16_t n) const {
        uint8_t r = 0;
        if (n > 0) {
            auto q = (uint8_t*) p;
            if (w) {
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
                    while (!SPI[SR](1)) {} // ~TXE
                    SPI.byte(DR) = 0;
                    while (!SPI[SR](0)) {} // ~RXNE
                    *q++ = SPI.byte(DR);
                }
                while (!SPI[SR](0)) {} // ~RXNE
                *q = SPI.byte(DR);
            }
        }
        return r;
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

    static constexpr IoReg<A>             SPI {};
    static constexpr IoReg<D>             DMA {};
    static constexpr IoReg<D+CHAN_STEP*T> DTX {}; // DMA channel TX
    static constexpr IoReg<D+CHAN_STEP*R> DRX {}; // DMA channel RX

    struct Config : BASE::Config {
        Irq txIrq, rxIrq;
        uint8_t dma, txReq, rxReq; // 0-based

        // TODO this is the same code in I2C and SPI
        void initDma () const {
            RCC(ena::DMA1+dma,1) = 1;

            // channel/stream/request setup (confusing naming differences!)
#if STM32G4 | STM32H7 | STM32WB | STM32WL
    #if STM32G4
            RCC(ena::DMAMUX, 1) = 1;
        #if STM32G431xx | STM32G441xx
            constexpr auto CHMAP = 6;
        #else
            constexpr auto CHMAP = 8;
        #endif
    #elif STM32WB | STM32WL
            constexpr auto CHMAP = 7;
    #elif STM32H7
            #define DMAMUX DMAMUX1
            constexpr auto CHMAP = 8;
    #endif
            DMAMUX[4*(CHMAP*dma+T)] = txReq;
            DMAMUX[4*(CHMAP*dma+R)] = rxReq;
#elif STM32L0 | STM32L4
            DMA[0xA8](4*T,4) = txReq; // CSELR
            DMA[0xA8](4*R,4) = rxReq; // CSELR
#endif

            // channel configuration
#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
            DTX[CCR] = 0b1001'0010; // MINC DIR TCIE
            DRX[CCR] = 0b1000'0010; // MINC TCIE
#elif STM32H7
            DTX[CCR] = 0b0100'0101'0000; // MINC DIR TCIE
            DRX[CCR] = 0b0100'0001'0000; // MINC TCIE
#else
            DTX[CCR] = (txReq<<25) | 0b0100'0101'0000; // CHSEL MINC DIR TCIE
            DRX[CCR] = (rxReq<<25) | 0b0100'0001'0000; // CHSEL MINC TCIE
#endif
        }
    };

    Config const cfg;

    Sync (Config const& c) : BASE (c.ena, c.mhz), Device ('S'), cfg (c) {}

    void init (char const* defs, int khz) {
        BASE::init(defs, khz);
        SPI[BASE::CR2](0,2) = 0b11; // RXDMAEN TXDMAEN

        cfg.initDma();

        // peripheral address config and interrupt vector setup
        DTX[CPAR] = A + BASE::DR;
        DRX[CPAR] = A + BASE::DR;

        irqInstall((uint8_t) cfg.txIrq);
        irqInstall((uint8_t) cfg.rxIrq);
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma, 1) = 0; // may be shared

    // sync version, dma with wfe
    uint8_t transfer (uint8_t w, uint8_t* p, uint16_t n) const {
        if (n == 0)
            return 0;

        startReq(w, p, n);
        while (DTX[CCR](0) || DRX[CCR](0)) // EN
            asm ("wfe");
        return finishReq(w, p, n);
    }

protected:
    constexpr static auto LEN_BITS = 14, LEN_MASK = (1<<(LEN_BITS+1)) - 1;

private:
    Chain msgs;

    void startReq (bool w, void* p, uint16_t n) const {
        assert(n > 0);

        if (w)
            cache::clean(p, n);

        DTX[CMAR] = (uintptr_t) p;
        DTX[CNDTR] = n;
        DTX[CCR](0) = 1; // EN

        if (!w) {
            DRX[CMAR] = (uintptr_t) p;
            DRX[CNDTR] = n;
            DRX[CCR](0) = 1; // EN
        }
    }

    uint8_t finishReq (bool w, void* p, uint16_t n) const {
        if (!w)
            cache::inval(p, n);
        uint8_t r;
        do
            r = SPI.byte(BASE::DR);
        while (SPI[BASE::SR](0)); // RXNE
        return r;
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
        mp->mLen = finishReq(mp->mTag, mp->mPtr, mp->mLen);
        reply(mp);
        if (!msgs.isEmpty())
            startAsync(*msgs.first());
    }

    void startAsync (Message& m) {
        if (m.mLen > 0)
            startReq(m.mTag, m.mPtr, m.mLen);
        else
            finish(); // this may be recursive
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

        return !msgs.isEmpty();
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Call : Sync<A,D,T,R> {
    using BASE = Sync<A,D,T,R>;
    using BASE::Sync; // constructor

    // async version, dma with sys::call
    uint8_t transfer (uint8_t w, uint8_t* p, uint16_t n) const {
        Message msg { BASE::dId, w, n, (uint8_t*) p };
        sys::call(msg); // async with thread suspend
        return msg.mLen;
    }
};

} // namespace jeeh
