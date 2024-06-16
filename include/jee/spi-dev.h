namespace jeeh {

// polled H/W version (see SpiGpio for bit-banged version)
template< uint32_t S >
struct SpiHw {
    static constexpr IoReg<S> SPI {};
    enum { CR1=0x00, CR2=0x04, SR=0x08, DR=0x0C }; // SPI regs

    struct Config {
        uint16_t ena;
        uint8_t mhz;
    };

    Config const dev;
    Pin nsel;

    SpiHw (Config const c) : dev (c) {}

    void enable () const { nsel = 0; }
    void disable () const { nsel = 1; }

    void init (char const* defs, int speed) {
        Pin pins [4]; // mosi, miso, nclk, nsel
        Pin::config(defs, pins, sizeof pins);
        nsel = pins[3];
        disable(); // start with NSEL high

        auto div = 0; // determine clock divider
        while ((dev.mhz >> (div+1)) > speed)
            ++div;

        RCC(dev.ena, 1) = 1;
        SPI[CR1] = (div<<3) | (1<<2); // BD MSTR
#if STM32F1 | STM32L0 | STM32L4
        SPI[CR2] = (1<<2); // SSOE
#else
        SPI[CR2] = (1<<12) | (7<<8) | (1<<2); // FRXTH DS SSOE
#endif
        SPI[CR1](6) = 1; // SPE
    }

    void deinit () {
        RCC(dev.ena, 1) = 0;
    }

    int transfer (int v) const {
        SPI.byte(DR) = v;
        while (SPI[SR](0) == 0) {} // RXNE
        return SPI.byte(DR);
    }

    void bufferIO (uint8_t* buf, uint16_t len, bool send) const {
        assert(len > 0U);
        enable();
        if (send) {
            SPI.byte(DR) = *buf++;
            while (--len > 0U) {
                while (SPI[SR](1) == 0) {} // TXE
                SPI.byte(DR) = *buf++;
                while (SPI[SR](0) == 0) {} // RXNE
                (void) +SPI.byte(DR);
            }
            while (SPI[SR](0) == 0) {} // RXNE
            (void) +SPI.byte(DR);
        } else {
            SPI.byte(DR) = 0;
            while (--len > 0U) {
                while (SPI[SR](1) == 0) {} // TXE
                SPI.byte(DR) = 0;
                while (SPI[SR](0) == 0) {} // RXNE
                *buf++ = SPI.byte(DR);
            }
            while (SPI[SR](0) == 0) {} // RXNE
            *buf = SPI.byte(DR);
        }
        disable();
    }
};

// DMA version, either sync-wfe or async (i.e. msgs sent to this device)
template< uint32_t S, uint32_t D, int T, int R >
struct SpiDma : SpiHw<S>, Device {
    using HW = SpiHw<S>;

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

    struct Config : SpiHw<S>::Config {
        Irq txIrq, rxIrq;
        uint8_t dma, txReq, rxReq; // 0-based
    };

    Config const dev;

    SpiDma (Config const& c) : SpiHw<S> (c), Device ('S'), dev (c) {}

    void init (char const* defs, int speed) {
        SpiHw<S>::init(defs, speed);
        HW::SPI[HW::CR2](0,2) = 0b11; // RXDMAEN TXDMAEN

        RCC(ena::DMA1+dev.dma, 1) = 1;
#if STM32L0 | STM32L4
        DMA[CSELR](4*T,4) = dev.txReq;
        DMA[CSELR](4*R,4) = dev.rxReq;
#endif
        DCT[CPAR] = S + HW::DR;
        DCR[CPAR] = S + HW::DR;
#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
        DCT[CCR] = 0b1001'0010; // MINC DIR TCIE
        DCR[CCR] = 0b1000'0010; // MINC TCIE
#elif STM32H7
        DCT[CCR] = 0b0100'0101'0000; // MINC DIR TCIE
        DCR[CCR] = 0b0100'0001'0000; // MINC TCIE
#else
        DCT[CCR] = (dev.txReq<<25) | 0b0100'0101'0000; // CHSEL MINC DIR TCIE
        DCR[CCR] = (dev.rxReq<<25) | 0b0100'0001'0000; // CHSEL MINC TCIE
#endif

        irqInstall((uint8_t) dev.txIrq);
        irqInstall((uint8_t) dev.rxIrq);
    }

    // void deinit () // RCC(ena::DMA1+dev.dma, 1) = 0; // may be shared

    // sync version, dma with wfe
    void bufferIO (uint8_t* buf, uint16_t len, bool send) const {
        startReq(send, buf, len);
        while (DCT[CCR](0) != 0 || DCR[CCR](0) != 0) // EN
            asm ("wfe");
        SpiHw<S>::disable();
        if (!send)
            cache::inval(buf, len);
    }

private:
    Chain msgs;

    void startReq (bool send, uint8_t* buf, uint16_t len) const {
        SpiHw<S>::enable();
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
            SpiHw<S>::disable();
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
        (void) +HW::SPI.byte(HW::DR);
        (void) +HW::SPI[HW::SR];
        return !msgs.isEmpty();
    }
};

template< uint32_t S, uint32_t D, int T, int R >
struct SpiDev : SpiDma<S,D,T,R> {
    using SpiDma<S,D,T,R>::SpiDma;

    void bufferIO (uint8_t* buf, uint16_t len, bool send) const {
        Message m { 'S', send ? 'W' : 'R', len, buf };
        sys::call(m); // async with thread suspend
    }
};

} // namespace jeeh
