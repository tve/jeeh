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
        RCC(dev.uena, 1) = 1;         // uart on
        baudRate(baud);

        rxBuf = sys::pool(RXBYTES+TXBYTES, cache::align);
        txBuf = rxBuf + RXBYTES;

        RCC(ena::DMA1+dev.dma, 1) = 1; // dma on

#if STM32WL
        RCC(ena::DMAMUX1, 1) = 1; // dma mux on
        uint16_t CxCR_rx = 4*((dev.dma*7)+dev.rxChan);
        DMAMUX[CxCR_rx] = dev.rxReq;
        uint16_t CxCR_tx = 4*((dev.dma*7)+dev.txChan);
        DMAMUX[CxCR_tx] = dev.txReq;
#else
        dmaReg(CSELR)(4*(dev.rxChan), 4) = dev.rxReq;
        dmaReg(CSELR)(4*(dev.txChan), 4) = dev.txReq;
#endif

        dmaRX(CNDTR) = RXBYTES;
        dmaRX(CMAR) = (uint32_t) rxBuf;
        dmaRX(CPAR) = dev.uart + RDR;
        dmaRX(CCR) = 0b1010'0111; // MINC CIRC HTIE TCIE EN

        dmaTX(CNDTR) = 0;
        dmaTX(CPAR) = dev.uart + TDR;
        dmaTX(CCR) = 0b1001'0010; // MINC DIR TCIE

        devReg(CR3) = 0b1100'0000; // DMAT DMAR
        devReg(CR1) = 0b0001'1101; // IDLEIE TE RE UE

        irqInstall((int) dev.idleIrq); // uart
        irqInstall((int) dev.rxIrq);   // dma rx
        irqInstall((int) dev.txIrq);   // dma tx
    }

    void deinit () {
        RCC(dev.uena, 1) = 0;          // uart off
        RCC(ena::DMA1+dev.dma, 1) = 0;  // dma off
    }

    void baudRate (uint32_t bd) const {
        auto n = SystemCoreClock;
        while (n > dev.mhz * 1'000'000)
            n /= 2;
        devReg(BRR) = n / bd;
    }

    void start (Message& m) override {
        switch (m.mTag) {
            case 'I':
                m.mLen = rxAvail();
                reply(&m);
                break;
            case 'R':
                if (m.mLen > 0) {
                    rxFill = (rxFill + m.mLen) % sizeof rxBuf;
                    if (m.mPtr == nullptr)
                        break;
                }
                m.mLen = rxAvail();
                if (m.mLen > 0) {
                    m.mPtr = rxBuf + rxFill;
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
        auto rxBit = 4*dev.rxChan;
        if (irq == (int) dev.idleIrq) {
            devReg(CR) = 0b0001'1111; // clear idle and error flags

            if (!rxMsgs.isEmpty() && rxAvail() > 0)
                return true; // rx done
        } else if (dmaReg(ISR)(rxBit)) {
            // can't use "if (irq == ...)" because on L0, rx & tx are same irq
            dmaReg(IFCR) = 1 << rxBit;

            if (!rxMsgs.isEmpty() && rxAvail() > 0)
                return true; // rx done
        } else if (irq == (int) dev.txIrq) {
            auto txBit = 4*dev.txChan;
            dmaReg(IFCR) = 1 << txBit;
            dmaTX(CCR)(0) = 0; // ~EN
            return true; // tx done
        }
        return false;
    }

    enum { CR1=0x00,CR3=0x08,BRR=0x0C,SR=0x1C,CR=0x20,RDR=0x24,TDR=0x28 };
    enum { ISR=0x00,IFCR=0x04,CCR=0x08,CNDTR=0x0C,CPAR=0x10,CMAR=0x14,CSELR=0xA8 };
    enum { CHAN_STEP=0x14 };
};

} // namespace jeeh
