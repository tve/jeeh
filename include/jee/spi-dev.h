namespace jeeh {

// polled H/W version (see SpiGpio for bit-banged version)
struct SpiHw {
    struct Config {
        uint32_t addr;
        uint16_t ena;
        uint8_t mhz;
    };

    Config const dev;
    volatile uint32_t* regs;
    Pin nsel;

    enum { CR1=0x00, CR2=0x04, SR=0x08, DR=0x0C }; // SPI regs
    auto& spiReg (int off) const { return regs[off/4]; }

    SpiHw (Config const c) : dev (c), regs ((volatile uint32_t*) c.addr) {}

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
        spiReg(CR1) = (div<<3) | (1<<2); // BD MSTR
#if STM32F1 | STM32L0 | STM32L4
        spiReg(CR2) = (1<<2); // SSOE
#else
        spiReg(CR2) = (1<<12) | (7<<8) | (1<<2); // FRXTH DS SSOE
#endif
        spiReg(CR1) |= 1<<6; // SPE
    }

    void deinit () {
        RCC(dev.ena, 1) = 0;
    }

    int transfer (int v) const {
        *(volatile uint8_t*) (dev.addr+DR) = v;
        while ((spiReg(SR) & 1) == 0) {} // RXNE
        return *(volatile uint8_t*) (dev.addr+DR);
    }

    void bufferIO (uint8_t* buf, uint16_t len, bool send) const {
        assert(len > 0U);
        auto& dr = *(volatile uint8_t*) (dev.addr+DR);

        enable();
        if (send) {
            dr = *buf++;
            while (--len > 0U) {
                while ((spiReg(SR) & 2) == 0) {} // TXE
                dr = *buf++;
                while ((spiReg(SR) & 1) == 0) {} // RXNE
                (void) +dr;
            }
            while ((spiReg(SR) & 1) == 0) {} // RXNE
            (void) +dr;
        } else {
            dr = 0;
            while (--len > 0U) {
                while ((spiReg(SR) & 2) == 0) {} // TXE
                dr = 0;
                while ((spiReg(SR) & 1) == 0) {} // RXNE
                *buf++ = dr;
            }
            while ((spiReg(SR) & 1) == 0) {} // RXNE
            *buf = dr;
        }
        disable();
    }
};

// DMA version, either sync-wfe or async (i.e. msgs sent to this device)
// unlike SpiHw, each block transfer also manages the SPI enable signal:
// - if mTag is 'L' (last), the NSEL pin is disabled on completion (high)
// - if mTag is 'M' (more), the NSEL pin is left enabled on completion (low)

struct SpiDma : SpiHw, Device {
    struct Config : SpiHw::Config {
        Irq txIrq, rxIrq;
        uint8_t dma :1, txChan :3, rxChan :3, txReq, rxReq; // 0-based
    };

    Config const dev;

#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STL32L4
    enum { ISR=0x00, IFCR=0x04,CCR=0x08,CNDTR=0x0C,CPAR=0x10,CMAR=0x14 };
    enum { CHAN_STEP=0x14 };
#else
    enum { IFCR=0x08,CCR=0x10,CNDTR=0x14,CPAR=0x18,CMAR=0x1C }; // DMA regs
    enum { CHAN_STEP=0x18 };
#endif
#if STM32L0 | STM32L4
    enum { CSELR=0xA8 };
#endif

    SpiDma (Config const& c) : SpiHw (c), Device ('S'), dev (c) {
        auto addrDma = DMA1.ADDR + 0x400*dev.dma;
        regDma = (volatile uint32_t*) addrDma;
        txDma = (volatile uint32_t*) (addrDma + CHAN_STEP*dev.txChan);
        rxDma = (volatile uint32_t*) (addrDma + CHAN_STEP*dev.rxChan);
    }

    auto& dmaReg (int off) const { return regDma[off/4]; }
    auto& dmaTX (int off) const { return txDma[off/4]; }
    auto& dmaRX (int off) const { return rxDma[off/4]; }

    void init (char const* defs, int speed) {
        SpiHw::init(defs, speed);
        spiReg(CR2) |= 0b11; // RXDMAEN TXDMAEN

        RCC(ena::DMA1+dev.dma, 1) = 1;
#if STM32L0 | STM32L4
        auto t = 4* dev.txChan, r = 4* dev.rxChan;
        dmaReg(CSELR) &= ~(0xF << t) & ~(0xF << r);
        dmaReg(CSELR) |= (dev.txReq << t) | (dev.rxReq << r);
#endif
        dmaTX(CPAR) = dev.addr + DR;
        dmaRX(CPAR) = dev.addr + DR;
#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
        dmaTX(CCR) = 0b1001'0010; // MINC DIR TCIE
        dmaRX(CCR) = 0b1000'0010; // MINC TCIE
#elif STM32H7
        dmaTX(CCR) = 0b0100'0101'0000; // MINC DIR TCIE
        dmaRX(CCR) = 0b0100'0001'0000; // MINC TCIE
#else
        dmaTX(CCR) = (dev.txReq<<25) | 0b0100'0101'0000; // CHSEL MINC DIR TCIE
        dmaRX(CCR) = (dev.rxReq<<25) | 0b0100'0001'0000; // CHSEL MINC TCIE
#endif

        irqInstall((uint8_t) dev.txIrq);
        irqInstall((uint8_t) dev.rxIrq);
    }

    // void deinit () // RCC(ena::DMA1+dev.dma, 1) = 0; // may be shared

    // sync version, dma with wfe
    void bufferIO (uint8_t* buf, uint16_t len, bool send) const {
        startReq(send, buf, len);
        while ((dmaTX(CCR) & 1) != 0 || (dmaRX(CCR) & 1) != 0) // EN
            asm ("wfe");
        disable();
        if (!send)
            cache::inval(buf, len);
    }

private:
    Chain msgs;
    volatile uint32_t* regDma;
    volatile uint32_t* txDma;
    volatile uint32_t* rxDma;

    void startReq (bool send, uint8_t* buf, uint16_t len) const {
        enable();
        if (!send) {
            dmaRX(CMAR) = (uint32_t) buf;
            dmaRX(CNDTR) = len;
            dmaRX(CCR) |= 1; // EN
        }
        // always send (RXIDLE mode is troublesome w/ DMA)
        cache::clean(buf, len);
        dmaTX(CMAR) = (uint32_t) buf;
        dmaTX(CNDTR) = len;
        dmaTX(CCR) |= 1; // EN
    }

    // async version, started from a msg
    void start (Message& m) override {
        if (!msgs.append(m))
            startReq(m.mTag == 'W', m.mPtr, m.mLen);
    }

    void finish () override {
        auto mp = msgs.pull();
        if (mp != nullptr) {
            disable();
            if (mp->mPtr != nullptr)
                cache::inval(mp->mPtr, mp->mLen);
            reply(mp);
        }
        mp = msgs.first();
        if (mp != nullptr)
            startReq(mp->mTag == 'W', mp->mPtr, mp->mLen);
    }

    bool interrupt (int) override {
        auto t = dev.txChan;
        auto r = dev.rxChan;
#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
        if (dmaReg(ISR) & (1 << (4*t))) { // GIF
            dmaTX(CCR) &= ~1; // ~EN
            dmaReg(IFCR) = 1<<(4*t);
        } else if (dmaReg(ISR) & (1 << (4*r))) { // GIF
            dmaRX(CCR) &= ~1; // ~EN
            dmaReg(IFCR) = 1<<(4*r);
        } else
            fail();
#else
        static uint8_t const ifcBits [] = { 0, 6, 16, 22 };
        if (dmaReg(t&~3) & (1 << (5+ifcBits[t&3]))) // tx TCIF
            dmaReg(IFCR+(t&~3)) = 0b111101 << ifcBits[t&3]; // clr irq
        else if (dmaReg(r&~3) & (1 << (5+ifcBits[r&3]))) // rx TCIF
            dmaReg(IFCR+(r&~3)) = 0b111101 << ifcBits[r&3]; // clr irq
        else
            fail();
#endif
        if ((dmaTX(CCR) & 1) || (dmaRX(CCR) & 1))
            return false; // still in progress

        // clear OVR flag, in case the data was never read
        (void) +spiReg(DR);
        (void) +spiReg(SR);
        return !msgs.isEmpty();
    }
};

struct SpiDev : SpiDma {
    using SpiDma::SpiDma;

    void bufferIO (uint8_t* buf, uint16_t len, bool send) const {
        Message m { 'S', send ? 'W' : 'R', len, buf };
        sys::call(m); // async with thread suspend
    }
};

} // namespace jeeh
