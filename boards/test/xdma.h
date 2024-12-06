// Common code for setting up the DMA channels.

namespace jeeh::dma {

template< typename T, T const& C >
struct DmaConfig {
#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
    enum { ISR=0x00,IFCR=0x04,CCR=0x08,CNDTR=0x0C,CPAR=0x10,CMAR=0x14 };
    enum { CHAN_STEP=0x14 };
#else
    enum { ISR=0x00,IFCR=0x08,CCR=0x10,CNDTR=0x14,CPAR=0x18,CMAR=0x1C };
    enum { CHAN_STEP=0x18 };
#endif
    enum { NONE, RXHALF, RXFULL, TXDONE };

    static constexpr IoReg<C.dmaAddr>             DMA {};
    static constexpr IoReg<C.dmaAddr+CHAN_STEP*C.dmaTs> DTX {}; // DMA channel TX
    static constexpr IoReg<C.dmaAddr+CHAN_STEP*C.dmaRs> DRX {}; // DMA channel RX

    void init (uint32_t txAddr, uint32_t rxAddr) const {
        RCC(ena::DMA1+C.dmaIdx,1) = 1;

        // channel/stream/request setup (confusing naming differences!)
#if STM32G4 | STM32H7 | STM32WB | STM32WL
#if STM32G4
        RCC(ena::DMAMUX,1) = 1;
    #if STM32G431xx | STM32G441xx
        constexpr auto CHMAP = 6;
    #else
        constexpr auto CHMAP = 8;
    #endif
#elif STM32H7
    #define DMAMUX DMAMUX1
        constexpr auto CHMAP = 8;
#else // STM32WB | STM32WL
        constexpr auto CHMAP = 7;
#endif
        DMAMUX[4*(CHMAP*C.dmaIdx+C.dmaTs)] = C.dmaTc;
        DMAMUX[4*(CHMAP*C.dmaIdx+C.dmaRs)] = C.dmaRc;
#elif STM32L0 | STM32L4
        DMA[0xA8](4*C.dmaTs,4) = C.dmaTc; // CSELR
        DMA[0xA8](4*C.dmaRs,4) = C.dmaRc; // CSELR
#endif

        // channel configuration
#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
        DTX[CCR] = 0b1001'0010; // MINC DIR TCIE
        DRX[CCR] = 0b1000'0010; // MINC TCIE
#elif STM32H7
        DTX[CCR] = 0b0100'0101'0000; // MINC DIR TCIE
        DRX[CCR] = 0b0100'0001'0000; // MINC TCIE
#else
        DTX[CCR] = (C.dmaTc<<25) | 0b0100'0101'0000; // CHSEL MINC DIR TCIE
        DRX[CCR] = (C.dmaRc<<25) | 0b0100'0001'0000; // CHSEL MINC TCIE
#endif

        DTX[CPAR] = txAddr;
        DRX[CPAR] = rxAddr;
    }

    void deinit () const {
        DTX[CCR] = 0;
        DRX[CCR] = 0;
    }

    void txStart (void const* p, uint16_t n) const {
        cache::clean(p, n);
        DTX[CMAR] = (uintptr_t) p;
        DTX[CNDTR] = n;
        DTX[CCR](0) = 1; // EN
    }

    void rxStart (void const* p, uint16_t n) const {
        DRX[CMAR] = (uintptr_t) p;
        DRX[CNDTR] = n;
        DRX[CCR](0) = 1; // EN
    }

    int completed () const {
#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
        if (DMA[ISR](4*C.dmaRs+2)) { // HTIF
            DMA[IFCR] = 1<<(4*C.dmaRs+2);
#if STM32F1 | STM32F3 | STM32G4
            if (DRX[CCR](5)) // only report if circular
#else
            if (DRX[CCR](8)) // only report if circular
#endif
                return RXHALF;
        }
        if (DMA[ISR](4*C.dmaRs)) { // GIF
            if (!DRX[CCR](5)) // only disable if not circular
                DRX[CCR](0) = 0; // ~EN
            DMA[IFCR] = 1<<(4*C.dmaRs);
            return RXFULL;
        }
        if (DMA[ISR](4*C.dmaTs)) { // GIF
            DTX[CCR](0) = 0; // ~EN
            DMA[IFCR] = 1<<(4*C.dmaTs);
            return TXDONE;
        }
#else
        constexpr uint8_t ifcBits [] = { 0, 6, 16, 22 };
        if ((uint8_t) DMA[C.dmaRs&~3](ifcBits[C.dmaRs&3],6)) { // rx irq
            auto d = DMA[C.dmaRs&~3](4+ifcBits[C.dmaRs&3]) ? RXHALF : RXFULL;
            DMA[IFCR+(C.dmaRs&~3)] = 0b111101 << ifcBits[C.dmaRs&3]; // clr irq
            return d;
        }
        if ((uint8_t) DMA[C.dmaTs&~3](ifcBits[C.dmaTs&3],6)) { // tx irq
            DMA[IFCR+(C.dmaTs&~3)] = 0b111101 << ifcBits[C.dmaTs&3]; // clr irq
            return TXDONE;
        }
#endif
        return NONE;
    }

    bool isRunning () const {
        return DTX[CCR](0) || DRX[CCR](0); // EN
    }

    void done () const {
        assert(isRunning());
        DTX[CCR](0) = 0; // ~EN
        DRX[CCR](0) = 0; // ~EN
    }
};

} // namespace jeeh
