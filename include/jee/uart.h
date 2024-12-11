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

} // namespace jeeh::uart
