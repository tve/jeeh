#include <strings.h>

//CG: board uartc

namespace jeeh {
    void fail (void const* a, char const* f, int n) {
        logf("\n" "failed at %s:%d\n"
                  "failed caller: %p", f, n, a);
#if MUST_FAIL
        logWriter("OK\n", 3);
#else
        logWriter("FAIL\n", 5);
#endif
        while (true) {}
    }

#ifdef UARTC_NAME


namespace console {

#if STM32F1 | STM32F4
enum { ISR=0x00, RDR=0x04, TDR=0x04, BRR=0x08, CR1=0x0C, UE=13 };
#else
enum { CR1=0x00, BRR=0x0C, ISR=0x1C, RDR=0x24, TDR=0x28, UE=0 };
#endif

void start () {
    Pin::config(UARTC_PINS);
    RCC(ena::UARTC_NAME, 1) = 1;
    UARTC_NAME[BRR] = SystemCoreClock / 115'200; // 0.5 Mbaud
    UARTC_NAME[CR1] = (1<<29) | (1<<3) | (1<<2) | (1<<UE);  // FIFOEN TE RE UE

    // wait briefly for an incoming '+' byte
    for (auto i = 0; UARTC_NAME[RDR] != '+' && i < 5; ++i)
        for (auto n = 0U; n < SystemCoreClock >> 4; ++n)
            if (UARTC_NAME[ISR](5)) // RXFNE
                break;
}

} // namespace console

void logWriter (void const* ptr, size_t len) {
    using namespace console;
    for (auto i = 0U; i < len; ++i) {
        while (!UARTC_NAME[ISR](7)) {} // TXFNF
        UARTC_NAME[TDR] = ((uint8_t const*) ptr)[i];
    }
}

#else
namespace console {
void start () {}
}
#endif // UARTC_NAME

} // namespace jeeh

struct Tester {
    Tester (char const* file =__builtin_FILE()) {
        extern uint32_t g_pfnVectors [];
        *(uint32_t**) 0xE000'ED08 = g_pfnVectors; // fix SCB->VTOR if in RAM

        fastClock();
        console::start();
#if SWO_FREQ
        swoInit(SWO_FREQ); // TODO openocd didn't init ITM/SWO on STM32WL
#endif
        auto p = rindex(file, '/');
        logf("\nTEST %s", p != nullptr ? p + 1 : file);
    }

    ~Tester () {
#if MUST_FAIL
        logf("Should have failed!\nFAIL");
#else
        logf("OK", 3);
#endif
        while (true) {} // FIXME crashes when Tester goes out of scape (?)
    }
};
