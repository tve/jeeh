namespace jeeh {

struct Uart : Device {
    struct Config {
        uint32_t uart;
        uint16_t uena;
        uint8_t mhz;
        Irq idleIrq, txIrq, rxIrq;
        uint8_t dma :1, txChan :3, rxChan :3, txReq, rxReq; // 0-based
    };

    auto devReg (int off) const { IoReg<0> io; return io[dev.uart+off]; }
    auto dmaReg (int off) const { return DMA1[0x400*dev.dma+off]; }
    auto dmaRX (int off) const { return dmaReg(off+CHAN_STEP*dev.rxChan); }
    auto dmaTX (int off) const { return dmaReg(off+CHAN_STEP*dev.txChan); }

    using Device::Device;

    void init (char const* pins, uint32_t baud, Config const& config) {
        Pin::config(pins);
        dev = config;
        RCC(dev.uena, 1) = 1;          // uart on
        baudRate(baud);

        static_assert(RXBYTES % cache::align == 0);
        static_assert(TXBYTES % cache::align == 0);
        rxBuf = sys::pool(RXBYTES+TXBYTES, nullptr, cache::align);
        txBuf = rxBuf + RXBYTES;

        RCC(ena::DMA1+dev.dma, 1) = 1; // dma on
#if STM32G4
        RCC(ena::DMAMUX, 1) = 1;
#elif STM32H7
#define DMAMUX DMAMUX1
#endif
#if STM32G4 | STM32H7
        DMAMUX[32*dev.dma+4*dev.rxChan] = dev.rxReq;
        DMAMUX[32*dev.dma+4*dev.txChan] = dev.txReq;
#endif

        dmaRX(CNDTR) = RXBYTES;
        dmaRX(CMAR) = (uint32_t) rxBuf;
        dmaRX(CPAR) = dev.uart + RDR;
#if STM32F1 | STM32F3 | STM32G4
        dmaRX(CCR) = 0b1010'0111; // MINC CIRC HTIE TCIE EN
#elif STM32H7
        dmaRX(CCR) = 0b0101'0001'1001; // MINC CIRC TCIE HTIE EN
#else
        dmaRX(CCR) = // CHSEL MINC CIRC TCIE HTIE EN
                    (dev.rxReq<<25) | 0b0101'0001'1001;
#endif

        dmaTX(CNDTR) = 0;
        dmaTX(CPAR) = dev.uart + TDR;
#if STM32F1 | STM32F3 | STM32G4
        dmaTX(CCR) = 0b1001'0010; // MINC DIR TCIE
#elif STM32H7
        dmaTX(CCR) = 0b0100'0101'0000; // MINC DIR TCIE
#else
        dmaTX(CCR) = (dev.txReq<<25) | 0b0100'0101'0000; // CHSEL MINC DIR TCIE
#endif

        devReg(CR3) = 0b1100'0000; // DMAT DMAR
#if STM32F1 | STM32F4
        devReg(CR1) = 0b0010'0000'0001'1100 ; // UE IDLEIE TE RE
#else
        devReg(CR1) = 0b0001'1101; // IDLEIE TE RE UE
#endif

        irqInstall((int) dev.idleIrq); // uart
        irqInstall((int) dev.rxIrq);   // dma rx
        irqInstall((int) dev.txIrq);   // dma tx
    }

    void deinit () {
        RCC(dev.uena, 1) = 0;          // uart off
        RCC(ena::DMA1+dev.dma, 1) = 0; // dma off
    }

    void baudRate (uint32_t bd) const {
        auto n = SystemCoreClock;
        while (n > dev.mhz * 1'000'000)
            n /= 2;
        devReg(BRR) = n / bd;
    }

    void start (Message& m) override {
        switch (m.mTag) {
            case 'R':
                if (m.mLen > 0) {
                    rxFill = (rxFill + m.mLen) % RXBYTES;
                    if (m.mPtr == nullptr)
                        break;
                }
                m.mLen = rxAvail();
                if (m.mLen > 0) {
                    m.mPtr = rxBuf + rxFill;
                    cache::inval(m.mPtr, m.mLen);
                    reply(&m);
                } else
                    rxMsgs.append(m);
                break;
            case 'W':
                if (!txMsgs.append(m))
                    txStart();
                break;
            default:
                m.mTag = -1;
                reply(&m);
        }
    }

    void finish () override {
        auto n = rxAvail();
        if (n > 0) {
            auto mp = rxMsgs.pull();
            if (mp != nullptr) {
                mp->mPtr = rxBuf + rxFill;
                mp->mLen = n;
                cache::inval(mp->mPtr, mp->mLen);
                reply(mp);
            }
        }
        txStart();
    }

    Config dev;
private:
    static constexpr auto RXBYTES = 128, TXBYTES = 128;
    uint8_t *rxBuf, *txBuf;
    uint16_t rxFill =0, txFill =0; // where the next data comes from / goes to
    Chain rxMsgs, txMsgs;

    uint32_t rxAvail () const {
        int n = RXBYTES - rxFill - dmaRX(CNDTR);
        return n >= 0 ?  n : RXBYTES - rxFill;
    }

    void txStart () {
        auto mp = txMsgs.first();
        if (mp == nullptr || dmaTX(CCR)(0))
            return;

        auto n = mp->mLen;
        if (n > TXBYTES - txFill)
            n = TXBYTES - txFill;
        assert(n > 0);

        dmaTX(CMAR) = (uint32_t) txBuf + txFill;
        dmaTX(CNDTR) = n;

        auto p = mp->mPtr;
        for (auto i = 0U; i < n; ++i)
            txBuf[txFill++] = *p++;

        cache::clean(txBuf + txFill - n, n);
        dmaTX(CCR)(0) = 1; // EN

        if (txFill >= TXBYTES)
            txFill = 0;
        mp->mPtr = p;
        mp->mLen -= n;
        if (mp->mLen == 0)
            reply(txMsgs.pull());
    }

    // the actual interrupt handler, with access to the uart object
    bool interrupt (int irq) override {
        [[maybe_unused]] static uint8_t const ifcBits [] = { 0, 6, 16, 22 };
        if (irq == (int) dev.idleIrq) {
#if STM32F1 | STM32F4
            (uint32_t) devReg(SR);
            (uint32_t) devReg(RDR); // clear idle and error flags
#else
            devReg(CR) = 0b0001'1111; // clear idle and error flags
#endif

            if (!rxMsgs.isEmpty() && rxAvail() > 0)
                return true; // rx done
        } else if (irq == (int) dev.rxIrq) {
            auto r = dev.rxChan;
#if STM32F1 | STM32F3 | STM32G4
            dmaReg(IFCR) = 1<<(4*r);
#else
            dmaReg(IFCR+(r&~3)) = 0b111101 << ifcBits[r&3];
#endif
            if (!rxMsgs.isEmpty() && rxAvail() > 0)
                return true; // rx done
        } else {
            auto t = dev.txChan;
#if STM32F1 | STM32F3 | STM32G4
            dmaReg(IFCR) = 1<<(4*t);
#else
            dmaReg(IFCR+(t&~3)) = 0b111101 << ifcBits[t&3];
#endif
            dmaTX(CCR)(0) = 0; // ~EN
            return true; // tx done
        }
        return false;
    }

#if STM32F1 | STM32F4
    enum { SR=0x00,RDR=0x04,TDR=0x04,BRR=0x08,CR1=0x0C,CR3=0x14 };
#else
    enum { CR1=0x00,CR3=0x08,BRR=0x0C,SR=0x1C,CR=0x20,RDR=0x24,TDR=0x28 };
#endif
#if STM32F1 | STM32F3 | STM32G4
    enum { IFCR=0x04,CCR=0x08,CNDTR=0x0C,CPAR=0x10,CMAR=0x14 };
    enum { CHAN_STEP=0x14 };
#else
    enum { IFCR=0x08,CCR=0x10,CNDTR=0x14,CPAR=0x18,CMAR=0x1C };
    enum { CHAN_STEP=0x18 };
#endif
};

} // namespace jeeh
