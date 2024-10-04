// Several UART variants (TODO: just poll/sync & tx-only for now).

namespace jeeh::uart {

template< uint32_t A >
struct Poll {
    static constexpr IoReg<A> UART {};
#if STM32F1 | STM32F4
    enum { SR=0x00,RDR=0x04,TDR=0x04,BRR=0x08,CR1=0x0C,CR3=0x14 };
#else
    enum { CR1=0x00,CR3=0x08,BRR=0x0C,SR=0x1C,CR=0x20,RDR=0x24,TDR=0x28 };
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

        UART[CR1] = (1<<3) | (1<<2) | (1<<0);  // TE RE UE
    }

    void deinit () {
        RCC (cfg.ena,1) = 0;
    }

    void baudRate (uint32_t bd) const {
        auto n = SystemCoreClock;
        while (n > cfg.mhz * 1'000'000)
            n /= 2;
        UART[CR1](0) = 0; // ~UE
        UART[BRR] = n / bd;
        UART[CR1](0) = 1; // UE
    }

    void transfer (bool w, uint8_t* p, uint16_t n) const {
        if (w)
            for (auto i = 0U; i < n; ++i) {
                while (!UART[SR](7)) {} // TXE
                UART[TDR] = *p;
if (*p++ == '\n' || 1) while (!UART[SR](6)) {} // ~TC FIXME
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
        uint8_t Xdma, XtxReq, XrxReq; // 0-based
    };

    DmaConfig<D,T,R> dma;
    Config const cfg;

    Sync (Config const& c)
        : BASE (c.ena, c.mhz),
          dma { c.Xdma, c.XtxReq, c.XrxReq }, cfg (c) {}

    void init (char const* defs, int baud) {
        BASE::init(defs, baud);
        UART[BASE::CR3](6,2) = 0b11; // DMAT DMAR

        // peripheral address config and interrupt vector setup
        dma.init(A + BASE::TDR, A + BASE::RDR);

        SCB[0x10](4) = 1; // SEVONPEND
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma, 1) = 0; // may be shared

    // sync version, dma with wfe
    void transfer (bool w, uint8_t* p, uint16_t n) const {
        if (n > 0) {
            startReq(w, p, n);
            while (!dma.completed() && dma.isRunning())
                asm ("wfe");
            Worker::irqClear(cfg.idleIrq);
            Worker::irqClear(cfg.txIrq);
            Worker::irqClear(cfg.rxIrq);
            finishReq(w, p, n);
        }
    }

protected:
    void startReq (bool w, void* p, uint16_t n) const {
        if (w)
            dma.txStart(p, n);
        else
            dma.rxStart(p, n);
    }

    void finishReq (bool w, void* p, uint16_t n) const {
        if (!w)
            cache::inval(p, n);
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Work : Sync<A,D,T,R>, Worker {
    using BASE = Sync<A,D,T,R>;
    using BASE::Sync, BASE::cfg, BASE::dma;

    enum TAG { RXIDLE, RXDONE, TXDONE };

    Event rxPending, txPending;

    uint8_t init (char const* defs, int baud) {
        BASE::init(defs, baud);
        irqEnable(cfg.idleIrq);
        irqEnable(cfg.rxIrq);
        irqEnable(cfg.txIrq);
        return Worker::init();
    }

    void deinit () {
        irqDisable(cfg.idleIrq);
        irqDisable(cfg.rxIrq);
        irqDisable(cfg.txIrq);
        BASE::deinit();
    }

    // async interface

    void write (void const* buf, uint16_t len, Event out) {
        assert(len > 0);
        txPending = out;
        BASE::startReq(true, (void*) buf, len);
    }

    void read (uint16_t skip, Event out) {
        if (!dma.DRX[dma.CCR](0)) { // start circular rx lazily
#if STM32F1 | STM32F3 | STM32G4
            dma.DRX[dma.CCR](5) = 1; // CIRC
#else
            dma.DRX[dma.CCR](8) = 1; // CIRC
#endif
            dma.rxStart(rxBuf, RX_MAX);
        }
#if 0 // TODO maybe ...
        if (skip == 0 && out.eDst == 0) { // flush input
            inPtr = rxBuf + RX_MAX - dma.DRX[dma.CNDTR];
            return;
        }
#endif
        inPtr = rxBuf + (inPtr - rxBuf + skip) % RX_MAX;
        out.eVal = rxAvail();
        if (out.eVal > 0)
            reply(out);
        else
            rxPending = out;
    }

    void interrupt () {
        if (auto f = dma.completed(); f > 0)
            trigger(TXDONE);
        else if (f < 0)
            trigger(RXDONE);
    }

    uint8_t const* inPtr = rxBuf;
private:
    enum { RX_MAX = 128 };
    uint8_t rxBuf [RX_MAX]; // TODO dynamic alloc & 32-byte cache-line aligned

    uint32_t rxAvail () const {
        auto cnt = dma.DRX[dma.CNDTR];
        assert(cnt > 0);
        auto end = (RX_MAX - cnt - 1) % RX_MAX + 1; // 1..RX_MAX
        auto pos = inPtr - rxBuf;
        return (end > pos ? end : RX_MAX) - pos;
    }

    Event process (Event in, Event out, void*) override {
        switch (in.eTag) {
            case RXDONE:
                rxPending.eVal = rxAvail();
                // TODO finishReq(w, p, n);
                reply(rxPending);
                rxPending = {};
                break;
            case TXDONE:
                // TODO finishReq(w, p, n);
                reply(txPending);
                txPending = {};
                break;
            default:
                fail();
        }
        return out;
    }
};

} // namespace jeeh::uart
