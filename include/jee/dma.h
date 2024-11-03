// Common code for setting up the DMA channels.

namespace jeeh {

template< uint32_t D, int T, int R >
struct DmaConfig {
#if STM32F1 | STM32F3 | STM32G4 | STM32L0 | STM32L4
    enum X { ISR=0x00, IFCR=0x04,CCR=0x08,CNDTR=0x0C,CPAR=0x10,CMAR=0x14 };
    enum { CHAN_STEP=0x14 };
#else
    enum { ISR=0x00, IFCR=0x08,CCR=0x10,CNDTR=0x14,CPAR=0x18,CMAR=0x1C };
    enum { CHAN_STEP=0x18 };
#endif
    enum { NONE, RXHALF, RXFULL, TXDONE };

    static constexpr IoReg<D>             DMA {};
    static constexpr IoReg<D+CHAN_STEP*T> DTX {}; // DMA channel TX
    static constexpr IoReg<D+CHAN_STEP*R> DRX {}; // DMA channel RX

    uint8_t idx, txReq, rxReq; // 0-based

    void init (uint32_t txAddr, uint32_t rxAddr) const {
        RCC(ena::DMA1+idx,1) = 1;

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
        DMAMUX[4*(CHMAP*idx+T)] = txReq;
        DMAMUX[4*(CHMAP*idx+R)] = rxReq;
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
//DTX[0x24](2) = 1; // DMDIS
//DRX[0x24](2) = 1; // DMDIS

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
        if (DMA[ISR](4*R+2)) { // HTIF
            DMA[IFCR] = 1<<(4*R+2);
#if STM32F1 | STM32F3 | STM32G4
            if (DRX[CCR](5)) // only report if circular
#else
            if (DRX[CCR](8)) // only report if circular
#endif
                return RXHALF;
        }
        if (DMA[ISR](4*R)) { // GIF
            if (!DRX[CCR](5)) // only disable if not circular
                DRX[CCR](0) = 0; // ~EN
            DMA[IFCR] = 1<<(4*R);
            return RXFULL;
        }
        if (DMA[ISR](4*T)) { // GIF
            DTX[CCR](0) = 0; // ~EN
            DMA[IFCR] = 1<<(4*T);
            return TXDONE;
        }
#else
        constexpr uint8_t ifcBits [] = { 0, 6, 16, 22 };
        if ((uint8_t) DMA[R&~3](ifcBits[R&3],6)) { // rx irq
            auto d = DMA[R&~3](4+ifcBits[R&3]) ? RXHALF : RXFULL;
            DMA[IFCR+(R&~3)] = 0b111101 << ifcBits[R&3]; // clr irq
            return d;
        }
        if ((uint8_t) DMA[T&~3](ifcBits[T&3],6)) { // tx irq
            DMA[IFCR+(T&~3)] = 0b111101 << ifcBits[T&3]; // clr irq
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
