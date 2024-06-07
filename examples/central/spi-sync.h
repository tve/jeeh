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

    enum { IFCR=0x08,CCR=0x10,CNDTR=0x14,CPAR=0x18,CMAR=0x1C }; // DMA regs
    enum { STREAM_STEP=0x18 };

    auto dmaReg (int off) const { return DMA1[0x400*dev.dma+off]; }
    auto dmaTX (int off) const { return dmaReg(off+STREAM_STEP*dev.txChan); }
    auto dmaRX (int off) const { return dmaReg(off+STREAM_STEP*dev.rxChan); }

    void init (char const* pins, int speed) {
        SpiGpio::init(pins);
        Pin::config(pins);

        auto div = 0; // determine clock divider
        while ((dev.mhz >> (div+1)) > speed)
            ++div;

        RCC(dev.ena, 1) = 1;
        devReg(CR1) = (div<<3) | (1<<2); // BD MSTR
        devReg(CR2) = (1<<12) | (7<<8) | (1<<2); // FRXTH DS SSOE
        devReg(CR1)(6) = 1; // SPE

        devReg(CR2)(0) = 1; // RXDMAEN
        devReg(CR2)(1) = 1; // TXDMAEN

        RCC(ena::DMA1+dev.dma, 1) = 1;
        dmaTX(CPAR) = dev.addr + DR;
        dmaTX(CCR) = (dev.txReq<<25) | 0b0100'0101'0000; // CHSEL MINC DIR TCIE
        dmaRX(CPAR) = dev.addr + DR;
        dmaRX(CCR) = (dev.rxReq<<25) | 0b0100'0001'0000; // CHSEL MINC TCIE

        SCB[0x10](4) = 1; // SEVONPEND
    }

    int transfer (int v) const {
        *(volatile uint8_t*) (dev.addr+DR) = v;
        while (devReg(SR)(0) == 0) {} // RXNE
        return *(volatile uint8_t*) (dev.addr+DR);
    }

    void transfer (uint8_t const* out, uint8_t* in, int len) const {
        assert(out != nullptr || in != nullptr);
if (out == nullptr) out = in; // TODO hack, don't know how to do RXONLY w/ DMA
        static uint8_t const ifcBits [] = { 0, 6, 16, 22 };

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
            while (dmaTX(CCR)(0)); // EN

            auto t = dev.txChan;
            dmaReg(IFCR+(t&~3)) = 0b111101 << ifcBits[t&3]; // clr irq
            auto n = (uint8_t) dev.txIrq;
            NVIC[0x180 + 4*(n/32)] = 1 << n%32; // clear pending
        }
        if (in != nullptr) {
            do
                asm ("wfe");
            while (dmaRX(CCR)(0)); // EN

            auto r = dev.rxChan;
            dmaReg(IFCR+(r&~3)) = 0b111101 << ifcBits[r&3]; // clr irq
            auto n = (uint8_t) dev.rxIrq;
            NVIC[0x180 + 4*(n/32)] = 1 << n%32; // clear pending
        } else { // clear OVR flag, as the data was never read
            (void) +devReg(DR);
            (void) +devReg(SR);
        }
    }

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
