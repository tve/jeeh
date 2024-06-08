struct SpiSync : Device, SpiGpio {
    struct Config {
        uint32_t addr;
        uint16_t ena;
        uint8_t mhz;
        Irq txIrq, rxIrq;
        uint8_t dma :1, txChan :3, rxChan :3, txReq, rxReq; // 0-based
    } const dev;

    struct Request : Message {
         uint8_t const* out;
         bool more;

         Request (uint8_t const* o, uint8_t* i, uint16_t n, bool m =false)
             : Message { 0, 'T', n, i }, out (o), more (m) {}
    };

    SpiSync (Config const& config) : Device ('S'), dev (config) {}

    enum { CR1=0x00, CR2=0x04, SR=0x08, DR=0x0C }; // SPI regs
    auto devReg (int off) const { IoReg<0> io; return io[dev.addr+off]; }

#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STL32L4
    enum { IFCR=0x04,CCR=0x08,CNDTR=0x0C,CPAR=0x10,CMAR=0x14 };
    enum { CHAN_STEP=0x14 };
#else
    enum { IFCR=0x08,CCR=0x10,CNDTR=0x14,CPAR=0x18,CMAR=0x1C }; // DMA regs
    enum { CHAN_STEP=0x18 };
#endif
#if STM32L0 | STM32L4
    enum { CSELR=0xA8 };
#endif

    auto dmaReg (int off) const { return DMA1[0x400*dev.dma+off]; }
    auto dmaTX (int off) const { return dmaReg(off+CHAN_STEP*dev.txChan); }
    auto dmaRX (int off) const { return dmaReg(off+CHAN_STEP*dev.rxChan); }

    void init (char const* pins, int speed) {
        SpiGpio::init(pins);
        Pin::config(pins);

        auto div = 0; // determine clock divider
        while ((dev.mhz >> (div+1)) > speed)
            ++div;

        RCC(dev.ena, 1) = 1;
        devReg(CR1) = (div<<3) | (1<<2); // BD MSTR
#if STM32F1 | STM32L0 | STM32L4
        devReg(CR2) = (1<<2); // SSOE
#else
        devReg(CR2) = (1<<12) | (7<<8) | (1<<2); // FRXTH DS SSOE
#endif
        devReg(CR2)(0) = 1; // RXDMAEN
        devReg(CR2)(1) = 1; // TXDMAEN
        devReg(CR1)(6) = 1; // SPE

        RCC(ena::DMA1+dev.dma, 1) = 1;
#if STM32L0 | STM32L4
        dmaReg(CSELR)(4*(dev.rxChan), 4) = dev.rxReq;
        dmaReg(CSELR)(4*(dev.txChan), 4) = dev.txReq;
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

        SCB[0x10](4) = 1; // SEVONPEND
    }

    int transfer (int v) const {
        *(volatile uint8_t*) (dev.addr+DR) = v;
        while (devReg(SR)(0) == 0) {} // RXNE
        return *(volatile uint8_t*) (dev.addr+DR);
    }

#if 0 // h/w version, polled
    void transfer (uint8_t const* out, uint8_t* in, int len) const {
        for (auto i = 0; i < len; ++i) {
            auto b = transfer(out != nullptr ? out[i] : 0);
            if (in != nullptr)
                in[i] = b;
        }
    }
#else
    void transfer (uint8_t const* out, uint8_t* in, int len) const {
        assert(out != nullptr || in != nullptr);
if (out == nullptr) out = in; // TODO hack, don't know how to do RXONLY w/ DMA
        [[maybe_unused]] static uint8_t const ifcBits [] = { 0, 6, 16, 22 };

        if (in != nullptr) {
            dmaRX(CMAR) = (uint32_t) in;
            dmaRX(CNDTR) = len;
            dmaRX(CCR)(0) = 1; // EN
        }
        if (out != nullptr) {
            dmaTX(CMAR) = (uint32_t) out;
            dmaTX(CNDTR) = len;
            dmaTX(CCR)(0) = 1; // EN

            do
                asm ("wfe");
            while (dmaTX(CNDTR) != 0);
#if STM32F1 | STM32L0 | STM32L4
            dmaTX(CCR)(0) = 0; // ~EN
#endif

#if STM32L0 // FIXME !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
logf("11"); //  leaving this out breaks SPI-DMA on L0 (UART is also using DMA!)
#endif

#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
            dmaReg(IFCR) = 1<<(4*dev.txChan);
#else
            auto t = dev.txChan;
            dmaReg(IFCR+(t&~3)) = 0b111101 << ifcBits[t&3]; // clr irq
#endif
            auto n = (uint8_t) dev.txIrq;
            NVIC[0x180 + 4*(n/32)] = 1 << n%32; // clear pending
        }
        if (in != nullptr) {
            do
                asm ("wfe");
            while (dmaRX(CNDTR) != 0);
#if STM32F1 | STM32L0 | STM32L4
            dmaRX(CCR)(0) = 0; // ~EN
#endif

#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
            dmaReg(IFCR) = 1<<(4*dev.rxChan);
#else
            auto r = dev.rxChan;
            dmaReg(IFCR+(r&~3)) = 0b111101 << ifcBits[r&3]; // clr irq
#endif
            auto n = (uint8_t) dev.rxIrq;
            NVIC[0x180 + 4*(n/32)] = 1 << n%32; // clear pending
        } else { // clear OVR flag, as the data was never read
            (void) +devReg(DR);
            (void) +devReg(SR);
        }
    }
#endif

    void transfer (Request const& req) const {
        enable();
        transfer(req.out, req.mPtr, req.mLen);
        if (!req.more)
            disable();
    }

private:
    void start (Message&) override {}
    void finish () override {}
    bool interrupt (int) override { return false; }
};
