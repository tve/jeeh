// Several UART variants: poll, sync, and work (also usable as sync).

namespace jeeh::uart {

template< uint32_t A >
struct Poll {
    static constexpr IoReg<A> UART {};
#if STM32F1 | STM32F4
    enum { SR=0x00,RDR=0x04,TDR=0x04,BRR=0x08,CR1=0x0C,CR3=0x14 };
#else
    enum { CR1=0x00,CR3=0x08,BRR=0x0C,SR=0x1C,ICR=0x20,RDR=0x24,TDR=0x28 };
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

        UART[CR1] = (1<<29) | (1<<3) | (1<<2) | (1<<0);  // FIFOEN TE RE UE
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

    void deinit () {
        dma.deinit();
        BASE::deinit();
    }

    // sync version, dma with wfe
    void transfer (bool w, uint8_t* p, uint16_t n) const {
        if (n > 0) {
            startReq(w, p, n);
            while (dma.completed() == 0 && dma.isRunning())
                asm ("wfe");
            Worker::irqClear(cfg.idleIrq);
            Worker::irqClear(cfg.txIrq);
            Worker::irqClear(cfg.rxIrq);
            if (!w)
                cache::inval(p, n);
        }
    }

protected:
    void startReq (bool w, void* p, uint16_t n) const {
        if (w)
            dma.txStart(p, n);
        else
            dma.rxStart(p, n);
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Work : Sync<A,D,T,R>, Worker {
    using BASE = Sync<A,D,T,R>;
    using BASE::Sync, BASE::cfg, BASE::dma, BASE::UART;

    enum TAG { RXIDLE, RXHALF, RXFULL, TXDONE };

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
            UART[BASE::CR1](4) = 1; // IDLEIE

#if STM32F1 | STM32F3 | STM32G4
            dma.DRX[dma.CCR](5) = 1; // CIRC
#else
            dma.DRX[dma.CCR](8) = 1; // CIRC
#endif
            dma.DRX[dma.CCR](2) = 1; // HTIE
            dma.rxStart(rxBuf, RX_MAX);
        }
        // TODO flush when skip is large, and deal with empty out
        rxPtr = rxBuf + (rxPtr - rxBuf + skip) % RX_MAX;
        out.eVal = rxAvail();
        if (out.eVal > 0)
            reply(out);
        else
            rxPending = out;
    }

    void idleIrq () {
#if STM32F1 | STM32F4
        +UART[BASE::SR];
        +UART[BASE::RDR]; // clear idle and error flags
#else
        UART[BASE::ICR] = 0x1F; // clear idle and error flags
#endif
        trigger(RXIDLE);
    }

    void dmaIrq () {
        auto f = dma.completed();
        if (f == dma.RXHALF)
            trigger(RXHALF);
        else if (f == dma.RXFULL)
            trigger(RXFULL);
        else if (f == dma.TXDONE)
            trigger(TXDONE);
    }

    uint8_t const* rxPtr = rxBuf;
private:
    enum { RX_MAX = 256 };
    uint8_t rxBuf [RX_MAX]; // TODO dynamic alloc & 32-byte cache-line aligned

    uint32_t rxAvail () const {
        auto end = RX_MAX - dma.DRX[dma.CNDTR];
        assert(0 <= end && end < RX_MAX);
        auto pos = rxPtr - rxBuf;
        assert(0 <= pos && pos < RX_MAX);
        return (end >= pos ? end : RX_MAX) - pos;
    }

    Event process (Event in, Event out, void*) override {
        switch (in.eTag) {
            case RXIDLE:
            case RXHALF:
            case RXFULL:
                if (rxPending)
                    if (auto n = rxAvail(); n > 0) {
                        cache::inval(rxPtr, n);
                        rxPending.eVal = n;
                        reply(rxPending);
                        rxPending = {};
                    }
                break;
            case TXDONE:
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
