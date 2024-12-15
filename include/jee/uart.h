// Several UART variants: poll, sync, and work (also usable as sync).

namespace jeeh::uart {

struct Config {
    char const* pins;           // gpio
    uint32_t base =0;           // poll
    uint16_t ena =0;
    uint8_t mhz =0;
    uint32_t dmaBase =0;        // sync
    uint8_t dmaIdx =0, dmaTs =0, dmaRs =0, dmaTc =0, dmaRc =0;
    Irq txIrq ={}, rxIrq ={}, idleIrq ={};
    uint16_t numIn =0;
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

        UART[CR1] = (1<<3) | (1<<2) | (1<<UE);  // TE RE UE
#if STM32G4 || STM32H7
        UART[CR1](29) = 1; // FIFOEN
#endif
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
        if (m & IO_WRITE)
            for (auto i = 0U; i < n; ++i) {
                while (!UART[SR](7)) {} // TXE
                UART[TDR] = p[i];
            }
        else
            for (auto i = 0U; i < n; ++i) {
                while (!UART[SR](5)) { // RXFNE
                    auto f = UART[SR] & 0x1F; // IDLE OVR NF FE PE
                    UART[ICR] = f; // clear idle and error flags
                    if (f & (1<<4)) // IDLE
                        return i;
                }
                p[i] = UART[RDR];
            }
        return n;
    }
};

template< Config const& C >
struct Sync : Poll<C>, IrqHandler {
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
        assert(!irqPending());
        if (n == 0)
            return 0; // TODO return # available for reading
        startReq(m, p, n);
        while (!checkIrq(m))
            asm ("wfe");
        return finishReq(m, p, n);
    }

protected:
    static inline bool checkIrq (uint16_t m) {
        if (m & IO_READ) {
            if (UART[BASE::SR](4)) { // IDLE
                UART[BASE::ICR] = 0x10; // IDLECF
                //(void) +UART[BASE::RDR];
                dma.rxDone(); // cancel dma, the line went idle
            } else if (dma.rxCompleted() == 0)
                return false;
dma.rxCompleted(); // TODO clears pending, but what's the logic here?
            irqClear(C.rxIrq);
            irqClear(C.idleIrq);
        } else {
            if (dma.txCompleted() == 0)
                return false;
dma.txCompleted(); // TODO clears pending, but what's the logic here?
            irqClear(C.txIrq);
        }
        return true;
    }

    void startReq (uint16_t m, void* p, uint16_t n) const {
        if (m & IO_READ) {
            UART[BASE::ICR] = 0x1F; // clear idle and error flags
            UART[BASE::CR1](4) = 1; // IDLEIE
            dma.rxStart(p, n);
        } else
            dma.txStart(p, n);
    }

    int finishReq (uint16_t m, void* p, uint16_t n) const {
        if (m & IO_READ) {
            n -= dma.DRX[dma.CNDTR]; // adjust receive count if IDLE detected
            cache::inval(p, n);
            UART[BASE::CR1](4) = 0; // ~IDLEIE
        }
        return n;
    }
};

template< Config const& C >
struct Async : Sync<C>, Task {
    using BASE = Sync<C>;
    using BASE::UART, BASE::dma;

    enum TAG { START, REQUEST, RXDONE, TXDONE };

    uint8_t const* rxPtr ={};

    uint8_t init (int hz =115'200) {
        BASE::init(hz);
        return Task::init();
    }

    void setReply (Event out) const {
        done = out; // TODO this can't handle simultaneous async tx & rx
    }

    int ioRequest (IoReq const* v, uint32_t n) const {
        if (!done)
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

    void irqTx () {
        auto f = BASE::checkIrq(IO_WRITE);
        assert(f);
        trigger(TXDONE);
    }

    void irqRx () {
        auto f = BASE::checkIrq(IO_READ);
        assert(f);
//#if STM32F1 | STM32F4
//        (void) +UART[BASE::SR];
//        (void) +UART[BASE::RDR]; // clear idle and error flags
//#else
//        UART[BASE::ICR] = 0x1F; // clear idle and error flags
//#endif
        trigger(RXDONE);
    }

private:
    mutable IoReq curr ={ 0, 0, nullptr };
    mutable IoReq const* reqs;
    mutable int num =0;
    mutable Event done;
    uint8_t rxBuf [C.numIn] alignas(4);

    Event process (Event in, Event out) override {
        assert(!out); // should use setReply instead
        switch (in.eTag) {
            case START:
                break;
            case REQUEST:
                while (--num >= 0) {
                    curr = *reqs++;
                    if (curr.mode & IO_READ) {
                        if (rxPtr == nullptr) { // lazily start circular rx
                            rxPtr = rxBuf;
                            UART[BASE::CR1](4) = 1; // IDLEIE
#if STM32F1 | STM32F3 | STM32G4
                            dma.DRX[dma.CCR](5) = 1; // CIRC
#else
                            dma.DRX[dma.CCR](8) = 1; // CIRC
#endif
                            dma.DRX[dma.CCR](2) = 1; // HTIE
                            dma.rxStart(rxBuf, sizeof rxBuf);
                        }
                        BASE::irqEnable(C.rxIrq);
                        BASE::irqEnable(C.idleIrq);
                    } else
                        BASE::irqEnable(C.txIrq);
                    BASE::startReq(curr.mode, curr.ptr, curr.len);
                    break; // transfer started, wait for a DONE trigger
            case RXDONE:   // this jumps back into the transfer loop!
            case TXDONE:   // ... and so does this
                    done.eVal = BASE::finishReq(curr.mode, curr.ptr, curr.len);
                    if (in.eTag == RXDONE) {
                        BASE::irqDisable(C.rxIrq);
                        BASE::irqDisable(C.idleIrq);
                    } else
                        BASE::irqDisable(C.txIrq);
                }
                out = take(done);
                break;
            default:
                fail();
        }
        return out;
    }
};

} // namespace jeeh::uart
