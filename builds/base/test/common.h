// Common code, used in all tests.

#include <unity.h>
#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
using namespace jeeh;

#undef assert
#define assert TEST_ASSERT

// tie printf and logf into Unity's output mechanism

extern "C" int _write (int, char* ptr, int len) {
    for (auto i = 0; i < len; ++i)
        putchar(ptr[i]);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

extern void allTests ();

int main () {
    fastClock();
    cycles::init();

    // adjust priorities before they might interfere with "real" IRQs
    SCB.byte(0x1F) = 0xFF; // irq #11: SVC
    SCB.byte(0x22) = 0xFF; // irq #14: PendSV

    UNITY_BEGIN();
    allTests();
    return UNITY_END();
}

// End of boilerplate, below is for temporary code, to be moved once ready.

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
        UART[BRR] = n / bd;
    }

    void transfer (bool w, void* p, uint16_t n) const {
        auto q = (uint8_t*) p;
        if (w)
            for (auto i = 0U; i < n; ++i) {
                while (!UART[SR](7)) {} // TXE
                UART[TDR] = *q;
// FIXME
if (*q++ == '\n' || 1)
    while (!UART[SR](6)) {} // ~TC
            }
    }
};

// DMA version, either sync-wfe or async (i.e. events sent to this device)
template< uint32_t A, uint32_t D, int T, int R >
struct Sync : Poll<A>, Worker {
    using BASE = Poll<A>;
    using BASE::Poll; // constructor

    enum TAG { DONE };
    constexpr static IoReg<A> UART {};

    struct Config : BASE::Config {
        Irq txIrq, rxIrq;
        uint8_t Xdma, XtxReq, XrxReq; // 0-based
    };

    DmaConfig<D,T,R> dma;
    Config const cfg;

    Sync (Config const& c, char const* name ="uart")
        : BASE (c.ena, c.mhz), Worker (name),
          dma { c.Xdma, c.XtxReq, c.XrxReq }, cfg (c) {}

    void init (char const* defs, int khz) {
        BASE::init(defs, khz);
        UART[BASE::CR3](6,2) = 0b11; // DMAT DMAR

        // peripheral address config and interrupt vector setup
        dma.init(A + BASE::TDR, A + BASE::RDR);

        irqEnable(cfg.txIrq);
        irqEnable(cfg.rxIrq);
    }

    // void deinit () // RCC(ena::DMA1+cfg.dma, 1) = 0; // may be shared

    // sync version, dma with wfe
    void transfer (bool w, void* p, uint16_t n) const {
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
