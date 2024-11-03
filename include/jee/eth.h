namespace jeeh {

struct Eth : Device {
    static constexpr auto DMA = ETHERNET_DMA;
    enum { BMR=0x00,TPDR=0x04,RPDR=0x08,RDLAR=0x0C,TDLAR=0x10,
           DSR=0x14,OMR=0x18,IER=0x1C };

    static constexpr auto MAC = ETHERNET_MAC;
    enum { CR=0x00,FFR=0x04,HTHR=0x08,HTLR=0x0C,MIIAR=0x10,MIIDR=0x14,
           FCR=0x18,MSR=0x38,IMR=0x3C,A0HR=0x40,A0LR=0x44 };

    uint16_t readPhy (int reg) {
        MAC[MIIAR] = (reg<<6) | (0b100<<2) | (1<<0);
        while (MAC[MIIAR](0)) {} // wait until MB clear, takes ≈ 32 µs
        return MAC[MIIDR];
    }
    void writePhy (int reg, uint16_t val) {
        MAC[MIIDR] = val;
        MAC[MIIAR] = (reg<<6) | (0b100<<2) | (1<<1) | (1<<0);
        while (MAC[MIIAR](0)) {} // wait until MB clear, takes ≈ 32 µs
    }

    struct DmaDesc {
        int32_t stat;
        uint32_t size;
        uint8_t* data;
        DmaDesc* next;
        uint32_t extStat, _gap, times [2];

        auto available () const {
            cache::inval(*this);
            return stat >= 0;
        }

        auto release () {
            cache::clean(*this);
            stat |= (1<<31);
            return next;
        }
    };
    static_assert(sizeof (DmaDesc) == 32);

    Chain txMsgs, rxMsgs;
    DmaDesc *txDone, *txFree, *rxDone, *rxFree;

    Eth (uint8_t id, uint32_t nDesc) : Device (id) {
        auto descs = sys::pool(nDesc * sizeof (DmaDesc), cache::align);
        assert(cache::align == 0 || (uint32_t) descs % cache::align == 0);

        auto txDesc = (DmaDesc*) descs;
        auto rxDesc = txDesc + nDesc/2;
        printf("  [eth] nDesc %d txDesc %08x rxDesc %08x\n",
                nDesc, txDesc, rxDesc);

        txDone = txFree = txDesc;
        rxDone = rxFree = rxDesc;

        for (auto i = 0U; i < nDesc/2; ++i) {
            txDesc[i].stat = (1<<20); // TCH, not owned by DMA
            txDesc[i].size = 0;
            txDesc[i].data = nullptr;
            txDesc[i].next = txDesc + (i+1) % (nDesc/2);

            rxDesc[i].stat = 0; // not owned by DMA
            rxDesc[i].size = (1<<14); // RCH
            rxDesc[i].data = nullptr;
            rxDesc[i].next = rxDesc + (i+1) % (nDesc/2);
        }
    }

    void init (uint8_t const mac [6]) {
        RCC(ena::ETHMAC,1) = 1;
        RCC(ena::ETHMACRX,1) = 1;
        RCC(ena::ETHMACTX,1) = 1;
        RCC(ena::SYSCFG,1) = 1;
        SYSCFG[0x04](23) = 1; // RMII_SEL in PMC

        DMA[BMR](0) = 1; // SR
        while (DMA[BMR](0)) {}

        // writePhy(0, 0x8000); // PHY reset
        // not set: MAC(FFR) MAC(HTHR) MAC(HTLR) MAC(FCR)

        DMA[BMR] = // AAB USP RDP FB PM PBL EDFE DA
            (1<<25) | (1<<24) | (1<<23) | (32<<17) |
            (1<<16) | (1<<14) | (32<<8) | (1<<7) | (1<<1);

        MAC[A0HR] = ((uint16_t const*) mac)[2];
        MAC[A0LR] = ((uint32_t const*) mac)[0];

        DMA[TDLAR] = (uint32_t) txDone;
        DMA[RDLAR] = (uint32_t) rxDone;

        DMA[OMR] = (1<<21) | (1<<20) | (1<<13) | (1<<1); // TSF FTF ST SR

        MAC[IMR] = (1<<9) | (1<<3); // TSTIM PMTIM
        DMA[IER] = (1<<16) | (1<<6) | (1<<0); // NISE RIE TIE

        irqInstall((uint8_t) Irq::ETH);
    }

    void checkLink () {
        if (!(readPhy(1) & (1<<2)))
            return; // link is down
        writePhy(0, 0x1000); // PHY auto-negotiation
        while ((readPhy(1) & (1<<5)) == 0) {} // wait for a-n complete
        auto r = readPhy(31);
        auto duplex = (r>>4) & 1, fast = (r>>3) & 1;

        MAC[CR] = // CSTF 15 FES DM IPCO APCS TE RE
            (1<<25) | (1<<15) | (fast<<14) | (duplex<<11) |
            (1<<10) | (1<<7) | (1<<3) | (1<<2);
    }

    void deinit () {
        SYSCFG[0x04](23) = 0; // ~RMII_SEL in PMC
        RCC[0x44](14) = 0; // ~SYSCFGEN in APB2ENR
        RCC[0x30](25, 3) = 0; // ETHMAC ~EN,~RXEN,~TXEN in AHB1ENR
    }

    void rxReply () {
        while (!rxMsgs.isEmpty() && rxDone->available()) {
            auto& m = *rxMsgs.pull();
            m.mLen = (rxDone->stat >> 16) & 0x3FFF;
            m.mPtr = take(rxDone->data);
            cache::inval(m.mPtr, m.mLen);
            reply(&m);
            rxDone = rxDone->next;
        }
    }

    void txReply () {
        while (!txMsgs.isEmpty() && txDone->available()) {
            auto& m = *txMsgs.pull();
            m.mLen = txDone->size & 0x1FFF;
            m.mPtr = take(txDone->data);
            //cache::inval(m.mPtr, m.mLen);
            reply(&m);
            txDone = txDone->next;
        }
    }

    void start (Message& m) override {
        switch (m.mTag) {
            case 'B': {
                if (MAC[CR](2) == 0) // ~RE if link is not yet up
                    checkLink();
                auto spares = (Chain*) m.mPtr;
                assert(spares != nullptr);
                while (!spares->isEmpty() && rxFree->data == nullptr) {
                    auto mp = spares->pull();
                    mp->mDst = m.mDst;
                    mp->mTag = 'R';
                    rxMsgs.append(*mp);
                    rxFree->data = mp->mPtr;
                    rxFree->size = (rxFree->size & ~0x1FFF) | mp->mLen;
                    rxFree = rxFree->release();
                    m.mLen++;
                }
                DMA[RPDR] = 0; // resume RX DMA
                break;
            }

            case 'W':
                assert(m.mPtr != nullptr && m.mLen > 0);
                assert(txFree->available() && txFree->data == nullptr);

                cache::clean(m.mPtr, m.mLen);
                txFree->stat = // IC LS FS CIC TCH
                    (0b0111<<28) | (3<<22) | (1<<20);
                txFree->data = m.mPtr;
                txFree->size = m.mLen;
                txFree = txFree->release();
                DMA[TPDR] = 0; // resume TX DMA

                if (!txMsgs.append(m))
                    txReply();
                break;

            default:
                m.mTag = -1;
                reply(&m);
        }
    }

    void finish () override {
        rxReply();
        txReply();
    }

    bool interrupt (int) override {
        DMA[DSR] = (1<<16) | (1<<6) | (1<<0); // clear NIS RS TS
        return true;
    }
};

} // namespace jeeh
