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
struct Sync : Poll<A>, Worker {
    using BASE = Poll<A>;

    enum TAG { DONE };
    constexpr static IoReg<A> UART {};

    struct Config : BASE::Config {
        Irq idleIrq, txIrq, rxIrq;
        uint8_t Xdma, XtxReq, XrxReq; // 0-based
    };

    DmaConfig<D,T,R> dma;
    Config const cfg;

    Sync (Config const& c, char const* name ="uart")
        : BASE (c.ena, c.mhz), Worker (name),
          dma { c.Xdma, c.XtxReq, c.XrxReq }, cfg (c) {}

    void init (char const* defs, int baud) {
        BASE::init(defs, baud);
        Worker::init();
        UART[BASE::CR3](6,2) = 0b11; // DMAT DMAR

        // peripheral address config and interrupt vector setup
        dma.init(A + BASE::TDR, A + BASE::RDR);

        irqEnable(cfg.idleIrq);
        irqEnable(cfg.txIrq);
        irqEnable(cfg.rxIrq);
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma, 1) = 0; // may be shared

    // sync version, dma with wfe
    void transfer (bool w, uint8_t* p, uint16_t n) const {
        if (n > 0) {
            startReq(w, p, n);
            while (dma.isRunning())
                asm ("wfe");
            finishReq(w, p, n);
        }
    }

    // async version, started from a msg
    void interrupt () {
        if (!dma.completed())
            fail();
        if (!dma.isRunning()) // other channel still in progress
            trigger(DONE);
    }

private:
    Event process (Event in, Event out, void* arg) override {
        (void) arg;
        switch (in.eTag) {
            case DONE:
                break;
            default:
                fail();
        }
        return out;
    }

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

} // namespace jeeh::uart
