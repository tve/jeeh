// see examples/uart/loop.cpp

namespace jeeh::uart {

template< uint32_t A >
struct Poll {
    using ID = uint8_t;

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

    void init (char const* defs, uint32_t baud) {
        Pin::config(defs);

        RCC(cfg.ena,1) = 1;
        baudRate(baud);

        UART[CR1] = (1<<3) | (1<<2) | (1<<0);  // TE RE UE
    }

    void deinit () {
        RCC(cfg.ena, 1) = 0;
    }

    void baudRate (uint32_t bd) const {
        auto n = SystemCoreClock;
        while (n > cfg.mhz * 1'000'000)
            n /= 2;
        UART[BRR] = n / bd;
    }

    void transfer (uint8_t w, void* p, uint16_t n) const {
        auto q = (uint8_t*) p;
        if (w)
            for (auto i = 0U; i < n; ++i) {
                while (!UART[SR](7)) {} // TXE
                UART[TDR] = *q++;
                while (UART[SR](6) == 0) {} // TC
            }
    }
};

// DMA version, either sync-wfe or async (i.e. msgs sent to this device)
template< uint32_t A, uint32_t D, int T, int R >
struct Sync : Poll<A>, Device {
    using BASE = Poll<A>;
    using BASE::Poll; // constructor

    static constexpr IoReg<A> UART {};

    struct Config : BASE::Config {
        Irq txIrq, rxIrq;
        uint8_t Xdma, XtxReq, XrxReq; // 0-based
    };

    DmaConfig<D,T,R> dma;
    Config const cfg;

    Sync (Config const& c)
        : BASE (c.ena, c.mhz), Device ('U'),
          dma { c.Xdma, c.XtxReq, c.XrxReq }, cfg (c) {}

    void init (char const* defs, int khz) {
        BASE::init(defs, khz);
        UART[BASE::CR3](6,2) = 0b11; // DMAT DMAR

        // peripheral address config and interrupt vector setup
        dma.init(A + BASE::TDR, A + BASE::RDR);

        irqInstall((uint8_t) cfg.txIrq);
        irqInstall((uint8_t) cfg.rxIrq);
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma, 1) = 0; // may be shared

    // sync version, dma with wfe
    void transfer (uint8_t w, uint8_t* p, uint16_t n) const {
        if (n == 0)
            return;

        startReq(w, p, n);
        while (dma.isRunning())
            asm ("wfe");
        finishReq(w, p, n);
    }

protected:
    constexpr static auto LEN_BITS = 14, LEN_MASK = (1<<(LEN_BITS+1)) - 1;

private:
    Chain msgs;

    void startReq (uint8_t w, void* p, uint16_t n) const {
        assert(n > 0);

        if (w)
            dma.txStart(p, n);
        else
            dma.rxStart(p, n);
    }

    void finishReq (uint8_t w, void* p, uint16_t n) const {
        if (!w)
            cache::inval(p, n);
    }

    // async version, started from a msg
    void start (Message& m) override {
        if (!msgs.append(m))
            startAsync(m);
    }

    void finish () override {
        auto mp = msgs.pull();
        if (mp == nullptr)
            return;
        finishReq(mp->mTag, mp->mPtr, mp->mLen);
        reply(mp);
        if (!msgs.isEmpty())
            startAsync(*msgs.first());
    }

    void startAsync (Message& m) {
        if (m.mLen > 0)
            startReq(m.mTag, m.mPtr, m.mLen);
        else
            finish(); // this may be recursive
    }

    bool interrupt (int) override {
        if (!dma.completed())
            fail();
        if (dma.isRunning())
            return false; // other channel still in progress

        return !msgs.isEmpty();
    }
};

template< uint32_t A, uint32_t D, int T, int R >
struct Call : Sync<A,D,T,R> {
    using BASE = Sync<A,D,T,R>;
    using BASE::Sync; // constructor

    // async version, dma with sys::call
    void transfer (uint8_t w, uint8_t* p, uint16_t n) const {
        Message msg { BASE::dId, w, n, (uint8_t*) p };
        sys::call(msg); // async with thread suspend
    }
};

} // namespace jeeh::uart
