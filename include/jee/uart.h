// Several UART variants: poll, sync, and work (also usable as sync).

namespace jeeh::uart {

struct Config {
    char const* pins;           // gpio
    uint32_t base =0;           // poll
    uint16_t ena =0;
    uint8_t mhz =0;
    uint32_t dmaBase =0;        // sync
    uint8_t dmaIdx =0, dmaTs =0, dmaRs =0, dmaTc =0, dmaRc =0;
    Irq txIrq ={}, rxIrq ={}, uartIrq ={};
};

template< Config const& C >
struct Poll {
    using IoSize = uint16_t;

    static constexpr IoReg<C.base> UART {};
#if STM32F1 | STM32F4
    enum { SR=0x00,RDR=0x04,TDR=0x04,BRR=0x08,CR1=0x0C,CR3=0x14,UE=13 };
#else
    enum { CR1=0x00,CR3=0x08,BRR=0x0C,SR=0x1C,ICR=0x20,RDR=0x24,TDR=0x28,UE=0 };
#endif

    Pin txPin, rxPin; // pin definitions must be kept in this order

    void init (int hz =115'200) {
        Pin::config(C.pins, &txPin, 2);

        RCC (C.ena,1) = 1;
        baudRate(hz);

        //UART[CR1] = (1<<29) | (1<<3) | (1<<2) | (1<<UE);  // FIFOEN TE RE UE
        UART[CR1] = (1<<3) | (1<<2) | (1<<UE);  // TE RE UE
    }

    void deinit () {
        RCC (C.ena,1) = 0;
        Pin::config(":F,", &txPin, 2);
    }

    void baudRate (uint32_t hz) const {
        auto n = SystemCoreClock;
        while (n > C.mhz * 1'000'000)
            n /= 2;
        UART[CR1](UE) = 0; // ~UE
        UART[BRR] = n / hz;
        UART[CR1](UE) = 1; // UE
    }

    int ioRequest (IoReq const* v, uint32_t n) const {
        int r = 0;
        for (auto i = 0U; i < n; ++i) {
            auto& t = v[i];
            r = ioRequest(t.mode, t.ptr, t.len);
            if (r < 0)
                break;
        }
        return r;
    }

    int ioRequest (uint16_t m, uint8_t* p, uint16_t n) const {
        if (m & IO_WRITE) {
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
        return n;
    }
};

template< Config const& C >
struct Sync : Poll<C> {
    using BASE = Poll<C>;
    using BASE::UART;

    static constexpr dma::DmaConfig<Config,C> dma {};

    void init (int hz =115'200) {
        BASE::init(hz);
        UART[BASE::CR3](6,2) = 0b11; // DMAT DMAR
        dma.init(C.base + BASE::TDR, C.base + BASE::RDR);
    }

    void deinit () {
        UART[BASE::CR2](6,2) = 0; // ~DMAT ~DMAR
        dma.deinit();
        BASE::deinit();
    }

    int ioRequest (IoReq const* v, uint32_t n) const {
        int r = 0;
        for (auto i = 0U; i < n; ++i) {
            auto& t = v[i];
            r = ioRequest(t.mode, t.ptr, t.len);
            if (r < 0)
                break;
        }
        return r;
    }

    int ioRequest (uint16_t m, uint8_t* p, uint16_t n) const {
        assert(!Task::pendingIrq());
        if (n > 0) {
            startReq(m, p, n);
            while (!Task::pendingIrq())
                asm ("wfe");
            dma.completed();
            assert(!dma.isRunning());
            Task::irqClear(C.uartIrq);
            Task::irqClear(C.txIrq);
            Task::irqClear(C.rxIrq);
            finishReq(m, p, n);
        }
        return n;
    }

protected:
    void startReq (uint16_t m, void* p, uint16_t n) const {
        if (m & IO_WRITE)
            dma.txStart(p, n);
        else
            dma.rxStart(p, n);
    }

    void finishReq (uint16_t m, void* p, uint16_t n) const {
        if (m & IO_READ)
            cache::inval(p, n);
    }
};

template< Config const& C >
struct Async : Sync<C>, Task {
    using BASE = Sync<C>;
    using BASE::dma;

    enum TAG { START, REQUEST, DONE };

    uint8_t init (int hz =115'200) {
        BASE::init(hz);
        return Task::init();
    }

    void setReply (Event out) const {
        pend = out;
    }

    int ioRequest (IoReq const* v, uint32_t n) const {
        if (!pend)
            return BASE::ioRequest(v, n); // use sync version
        reqs = v;
        num = n;
        send({ tId, REQUEST });
        return 0;
    }

    int ioRequest (uint16_t m, uint8_t* p, uint16_t n) const {
        curr = { m, n, p };
        return ioRequest(&curr, 1);
    }

    void irqDma () {
        auto f = dma.completed();
        assert(f > 0);
        trigger(DONE);
    }

    void irqIdle () {
        BASE::UART[BASE::ICR] = (1<<4); // IDLECF
        trigger(DONE);
    }

private:
    mutable IoReq curr ={ 0, 0, nullptr };
    mutable IoReq const* reqs;
    mutable int num =0;
    mutable Event pend;

    Event process (Event in, Event out) override {
        assert(!out); // should use setReply instead
        switch (in.eTag) {
            case START:
                break;
            case REQUEST:
                irqEnable(C.uartIrq);
                irqEnable(C.txIrq);
                irqEnable(C.rxIrq);
                while (--num >= 0) {
                    curr = *reqs++;
                    BASE::startReq(curr.mode, curr.ptr, curr.len);
                    break; // transfer started, wait for DONE trigger
            case DONE:     // this jumps back into the transfer loop!
                    BASE::finishReq(curr.mode, curr.ptr, curr.len);
                    pend.eVal = curr.len;
                }
                irqDisable(C.uartIrq);
                irqDisable(C.txIrq);
                irqDisable(C.rxIrq);
                out = take(pend);
                break;
            default:
                fail();
        }
        return out;
    }
};

} // namespace jeeh::uart
