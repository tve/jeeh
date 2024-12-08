// Several UART variants: poll, sync, and work (also usable as sync).

namespace jeeh::uart {

template< uint32_t A >
struct Poll {
    static constexpr IoReg<A> UART {};
#if STM32F1 | STM32F4
    enum { SR=0x00,RDR=0x04,TDR=0x04,BRR=0x08,CR1=0x0C,CR3=0x14,UE=13 };
#else
    enum { CR1=0x00,CR3=0x08,BRR=0x0C,SR=0x1C,ICR=0x20,RDR=0x24,TDR=0x28,UE=0 };
#endif

    struct Config {
        uint16_t ena;
        uint8_t mhz;
    };

    Config const cfg;

    Poll (uint16_t e, uint8_t f) : cfg { e, f } {}
    ~Poll () { RCC (cfg.ena,1) = 0; }

    void init (char const* defs, uint32_t baud) {
        Pin::config(defs);

        RCC (cfg.ena,1) = 1;
        baudRate(baud);

        //UART[CR1] = (1<<29) | (1<<3) | (1<<2) | (1<<UE);  // FIFOEN TE RE UE
        UART[CR1] = (1<<3) | (1<<2) | (1<<UE);  // TE RE UE
    }

    void deinit () {
        RCC (cfg.ena,1) = 0;
    }

    void baudRate (uint32_t bd) const {
        auto n = SystemCoreClock;
        while (n > cfg.mhz * 1'000'000)
            n /= 2;
        UART[CR1](UE) = 0; // ~UE
        UART[BRR] = n / bd;
        UART[CR1](UE) = 1; // UE
    }

    void transfer (bool w, uint8_t* p, uint16_t n) const {
        if (w) {
            for (auto i = 0U; i < n; ++i) {
                while (!UART[SR](7)) {} // TXE
                UART[TDR] = p[i];
            }
            while (!UART[SR](6)) {} // ~TC
        } else
            for (auto i = 0U; i < n; ++i) {
                while ((UART[SR] & 0x2F) == 0) {} // ~RXNE ~OVR ~NF ~FE ~PE
                p[i] = UART[RDR];
            }
    }
};

// DMA version, either sync-wfe or async (i.e. events sent to this device)
template< uint32_t A, uint32_t D, int T, int R >
struct Sync : Poll<A> {
    using BASE = Poll<A>;

    constexpr static IoReg<A> UART {};

    struct Config : BASE::Config {
        Irq idleIrq, txIrq, rxIrq;
        DmaConfig<D,T,R> dma;
    };

    Config const cfg;

    Sync (Config const& c) : BASE (c.ena, c.mhz), cfg (c) {}

    void init (char const* defs, int baud) {
        BASE::init(defs, baud);
        UART[BASE::CR3](6,2) = 0b11; // DMAT DMAR

        // peripheral address config and interrupt vector setup
        cfg.dma.init(A + BASE::TDR, A + BASE::RDR);

        SCB[0x10](4) = 1; // SEVONPEND
    }

    void deinit () {
        cfg.dma.deinit();
        BASE::deinit();
    }

    // sync version, dma with wfe
    void transfer (bool w, uint8_t* p, uint16_t n) const {
        if (n > 0) {
            startReq(w, p, n);
            while (true) {
                if (!cfg.dma.isRunning())
                    break;
                if (cfg.dma.completed() == 0)
                    asm ("wfe");
            }
            cfg.dma.completed();
            Task::irqClear(cfg.idleIrq);
            Task::irqClear(cfg.txIrq);
            Task::irqClear(cfg.rxIrq);
            finishReq(w, p, n);
        }
    }

protected:
    void startReq (bool w, void* p, uint16_t n) const {
        if (w)
            cfg.dma.txStart(p, n);
        else
            cfg.dma.rxStart(p, n);
    }

    void finishReq (uint8_t w, void* p, uint16_t n) const {
        if (w)
            while (!UART[BASE::SR](6)) {} // ~TC
        else            
            cache::inval(p, n);
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Async : Sync<A,D,T,R>, Task {
    using BASE = Sync<A,D,T,R>;
    using BASE::Sync, BASE::cfg, BASE::UART;

    enum TAG { START, RXHALF, RXFULL, TXDONE, RXIDLE }; // see irqDma()

    Event rxPending, txPending;

    uint8_t init (char const* defs, int baud) {
        BASE::init(defs, baud);
        irqEnable(cfg.idleIrq);
        irqEnable(cfg.rxIrq);
        irqEnable(cfg.txIrq);
        return Task::init();
    }

    void deinit () {
        irqDisable(cfg.idleIrq);
        irqDisable(cfg.rxIrq);
        irqDisable(cfg.txIrq);
        BASE::deinit();
    }

    // TODO maybe keep this as wrapper for the blocking variants?
    void transfer (bool w, uint8_t* p, uint16_t n) const =delete;

    // async interface

    void write (void const* buf, uint16_t len, Event out ={}) {
        assert(len > 0);
        txPending = out;
        txPending.eVal = len;
        BASE::startReq(true, (void*) buf, len);
        if (!out) { // blocking mode
            while (cfg.dma.DTX[cfg.dma.CCR](0))
                asm ("wfi");
            BASE::finishReq(true, (void*) buf, len);
        }
    }

    void read (uint16_t skip, Event out) {
        if (!cfg.dma.DRX[cfg.dma.CCR](0)) { // start circular rx lazily
            UART[BASE::CR1](4) = 1; // IDLEIE

#if STM32F1 | STM32F3 | STM32G4
            cfg.dma.DRX[cfg.dma.CCR](5) = 1; // CIRC
#else
            cfg.dma.DRX[cfg.dma.CCR](8) = 1; // CIRC
#endif
            cfg.dma.DRX[cfg.dma.CCR](2) = 1; // HTIE
            cfg.dma.rxStart(rxBuf, RX_MAX);
        }
        // TODO flush when skip is large, and deal with empty out
        rxPtr = rxBuf + (rxPtr - rxBuf + skip) % RX_MAX;
        out.eVal = rxAvail();
        if (out.eVal > 0)
            reply(out);
        else
            rxPending = out;
    }

    void irqIdle () {
#if STM32F1 | STM32F4
        (void) +UART[BASE::SR];
        (void) +UART[BASE::RDR]; // clear idle and error flags
#else
        UART[BASE::ICR] = 0x1F; // clear idle and error flags
#endif
        trigger(RXIDLE);
    }

    void irqDma () {
        auto f = cfg.dma.completed();
        assert(f != 0);
        static_assert(cfg.dma.RXHALF == (int) RXHALF);
        static_assert(cfg.dma.RXFULL == (int) RXFULL);
        static_assert(cfg.dma.TXDONE == (int) TXDONE);
        trigger(f);
    }

    uint8_t const* rxPtr = rxBuf;
private:
    enum { RX_MAX = 64 };
    uint8_t rxBuf [RX_MAX]; // TODO dynamic alloc & 32-byte cache-line aligned

    uint32_t rxAvail () const {
        auto end = RX_MAX - cfg.dma.DRX[cfg.dma.CNDTR];
        assert(0 <= end && end < RX_MAX);
        auto pos = rxPtr - rxBuf;
        assert(0 <= pos && pos < RX_MAX);
        return (end >= pos ? end : RX_MAX) - pos;
    }

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                break;
            case RXIDLE:
            case RXHALF:
            case RXFULL:
                if (rxPending)
                    if (auto n = rxAvail(); n > 0) {
                        cache::inval(rxPtr, n);
                        rxPending.eVal = n;
                        reply(take(rxPending));
                    }
                break;
            case TXDONE:
                reply(take(txPending));
                break;
            default:
                fail();
        }
        return out;
    }
};

} // namespace jeeh::uart
