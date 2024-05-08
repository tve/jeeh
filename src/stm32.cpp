#include "jee.h"

#if STM32

namespace jeeh {

#if STM32F1
#include "arch/stm32f1.h"
#elif STM32F3
#include "arch/stm32f3.h"
#elif STM32F4
#include "arch/stm32f4.h"
#elif STM32F7
#include "arch/stm32f7.h"
#elif STM32G0
#include "arch/stm32g0.h"
#elif STM32G4
#include "arch/stm32g4.h"
#elif STM32H7
#include "arch/stm32h7.h"
#elif STM32L0
#include "arch/stm32l0.h"
#elif STM32L4
#include "arch/stm32l4.h"
#endif

} // namespace jeeh

using namespace jeeh;

//------------------------------------------------------------------------ ITM

#if !STM32G0 && !STM32L0 // Cortex M0+ doesn't support ITM

void jeeh::itmWrite (void const* ptr, size_t len) {
    constexpr IoReg<0xE000'0000> ITM;
    enum { TER=0xE00, TCR=0xE80 };

    if (ITM[TCR](0) && ITM[TER](0)) { // ITM and channel 0 both enabled
        auto pos = (uintptr_t) ptr;
        while (len > 0) {
            while (!ITM[0](0)) {}
            int step = pos % 4 == 0 && len >= 4 ? 4 : 1;
            if (step == 4)
                ITM[0] = *(uint32_t const*) pos;
            else
                ITM.byte(0) = *(uint8_t const*) pos;
            pos += step;
            len -= step;
        }
    }
}

#endif // !STM32L0

//--------------------------------------------------------------------- Ticker

inline namespace {

struct Ticker : Device, Chain {
    volatile uint32_t ticks;
    uint16_t rate;

    Ticker () : Device (Device::BASE), ticks (0), rate (0) {
        SCB.byte(0x23) = 0xFF; // irq #15: lowest IRQ priority
    }

    void init () {
        auto ticksPerMs = SystemCoreClock / 1000;
#if STM32G4
        if (SystemCoreClock > 150'000'000) // TODO use actual HPRE divider
            ticksPerMs /= 2; // HPRE is set to 2 (AHB freq must be <= 150 MHz)
#endif

        uint16_t next = cHead->mLen - ticks;
        rate = next < 100 ? next : 100;

        STK[0x4] = (rate * ticksPerMs) / 8 - 1; // reload value
        STK[0x0] = 0b011;                       // enable, clk/8 mode
    }

    void start (Message& msg) override {
        auto ms = msg.mLen;
        assert(ms <= 60'000);

        if (!isEmpty()) {
            auto next = (uint16_t) (cHead->mLen - ticks);
            if (ms > next)
                ms = next;
        }

        if (ms < rate) {
            ticks = millis(); // update actual tick count
            STK[0x0] = 0;     // stop the clock
        }

        auto t = ticks;
        auto pp = &cHead; // insert in proper position
        while (*pp != nullptr && msg.mLen >= (uint16_t) ((*pp)->mLen - t))
            pp = &(*pp)->mLnk;

        msg.mLen += t; // make absolute, truncated to 16 bits
        msg.mLnk = *pp;
        *pp = &msg;

        finish();
    }

    void finish () override {
        while (expired())
            reply(pull());
        if (isEmpty())
            STK[0x0] = 0; // disable
        else
            init();
    }

    bool interrupt (int) override {
        ticks += rate;
        assert(cHead != nullptr);
        uint16_t next = cHead->mLen - ticks;
        return next < rate || next > 60'000;
    }

    bool expired () const {
        return !isEmpty() && (uint16_t) (cHead->mLen - ticks - 1) > 60'000;
    }

    uint32_t millis () const {
        // the result has millisecond resolution, even when rate > 1
        while (true) // spinloop, in case ticks changes midway
            if (uint32_t t = ticks, n = STK[0x08]; t == ticks)
                return t - (n*8)/(SystemCoreClock/1000);
    }
};

} // namespace inline

extern "C"
void SysTick_Handler () {
    Device::byId(Device::BASE).irqTrigger(0);
}

void sys::wait (uint16_t ms) {
    static Ticker ticker;
    Message m { ticker.dId, 'T', ms };
    call(m);
}

#if !STM32G0 & !STM32L0 & !STM32F1
namespace jeeh::rtc {

enum { TR=0x00,DR=0x04,SSR=0x08,ICSR=0x0C,WUTR=0x14,
        CR=0x18,WPR=0x24,SCR=0x5C,BKPR=0x50 };

#if STM32F3
enum { BDCR=0x20 };
#elif STM32F4 | STM32F7 | STM32H7
enum { BDCR=0x70 };
#elif STM32G4 | STM32L4
enum { BDCR=0x90 };
#endif

void init (bool lse) {
#if !STM32H7
    RCC(ena::PWR, 1) = 1;
#endif
    PWR[0x00](8) = 1; // DBP

    if (lse) {
#if STM32F723xx
        RCC[BDCR](3,2) = 1;           // LSEDRV (needed on f723d)
#endif
        RCC[BDCR](0) = 1;             // LSEON backup domain
        while (RCC[BDCR](1) == 0) {}  // wait for LSERDY
        RCC[BDCR](8,2) = 1;           // RTSEL = LSE
    } else
        RCC[BDCR](8,2) = 2;           // RTSEL = LSI
    RCC[BDCR](15) = 1;                // RTCEN

    RTC[WPR] = 0xCA;  // disable write protection, [1] p.803
    RTC[WPR] = 0x53;
    RTC[CR](5) = 1;   // BYPSHAD, this is faster that waiting for RSF
    RTC[WPR] = 0xFF;  // re-enable write protection
}

void deepSleep (uint16_t ms, int mode) {
    assert(ms <= 16'000);
    auto sel = 3;
    auto count = (1000*ms) / 61;
    while (count >= 32768) {
        --sel;
        count /= 2;
    }

    // see RM0440 v7 p1545
    RTC[WPR] = 0xCA;             // disable write protection
    RTC[WPR] = 0x53;
    RTC[CR](10) = 0;             // ~WUTE
    while (RTC[ICSR](2) == 0) {} // wait for WUTWF
    RTC[WUTR] = count;
    RTC[CR](0,3) = sel;
    RTC[CR](14) = 1;             // WUTIE
    RTC[SCR] = 1<<2;             // CWUTF
    RTC[CR](10) = 1;             // WUTE
    RTC[WPR] = 0xFF;             // re-enable write protection

    EXTI[0x08](20) = 1; // RT20 in RTSR1
    EXTI[0x04](20) = 1; // EM20 in EMR1

    PWR[0x00](0, 3) = mode; // CR1: LPMS
    SCB[0x10](2) = 1; // SLEEPDEEP
    asm ("wfe");
}

DateTime getDate () {
    uint32_t ssr, tod, doy;
    do { // loop until SSR is stable during all reads
        ssr = RTC[SSR];
        tod = RTC[TR];
        doy = RTC[DR];
    } while ((int) ssr != RTC[SSR]);

    DateTime dt;
    dt.ff = 255 - ssr; // assumes PREDIV_S is 255
    dt.ss = (tod & 0xF) + 10 * ((tod>>4) & 0x7);
    dt.mm = ((tod>>8) & 0xF) + 10 * ((tod>>12) & 0x7);
    dt.hh = ((tod>>16) & 0xF) + 10 * ((tod>>20) & 0x3);
    dt.dy = (doy & 0xF) + 10 * ((doy>>4) & 0x3);
    dt.mo = ((doy>>8) & 0xF) + 10 * ((doy>>12) & 0x1);
    // works until end 2063, will fail (i.e. roll over) in 2064 !
    dt.yr = ((doy>>16) & 0xF) + 10 * ((doy>>20) & 0x7);
    return dt;
}

uint32_t getSecs () {
    return getDate(); // let DateTime::operator uint32_t do the conversion
}

void set (DateTime const& dt) {
    RTC[WPR] = 0xCA;  // disable write protection, [1] p.803
    RTC[WPR] = 0x53;

    RTC[ICSR](7) = 1;             // set INIT
    while (RTC[ICSR](6) == 0) {}  // wait for INITF
    RTC[TR] = (dt.ss + 6 * (dt.ss/10)) |
        ((dt.mm + 6 * (dt.mm/10)) << 8) |
        ((dt.hh + 6 * (dt.hh/10)) << 16);
    RTC[DR] = (dt.dy + 6 * (dt.dy/10)) |
        ((dt.mo + 6 * (dt.mo/10)) << 8) |
        ((dt.yr + 6 * (dt.yr/10)) << 16);
    RTC[ICSR](7) = 0;             // clear INIT

    RTC[WPR] = 0xFF;  // re-enable write protection
}

uint32_t getReg (int reg) {
#if STM32G4
    return TAMP[0x100+4*reg]; // regs 0..31
#else
    return RTC[BKPR+4*reg];   // regs 0..31
#endif
}

void setReg (int reg, uint32_t val) {
#if STM32G4
    TAMP[0x100+4*reg] = val;  // regs 0..31
#else
    RTC[BKPR+4*reg] = val;    // regs 0..31
#endif
}

} // namespace jeeh::rtc
#endif // !STM32G0 & !STM32L0 & !STM32F1

namespace jeeh::dog {

enum { KR=0x00, PR=0x04, RLR=0x08, SR=0x0C };

uint32_t cause;

int resetCause () {
#if STM32F1 | STM32F3
    enum { CSR=0x24, RMVF=24 };
#elif STM32F4 | STM32F7
    enum { CSR=0x74, RMVF=24 };
#elif STM32H7
    enum { CSR=0xD0, RMVF=16 };
#elif STM32G4 | STM32G0 | STM32L0 | STM32L4
    enum { CSR=0x94, RMVF=23 };
#endif
    if (cause == 0) {
        cause = RCC[CSR];
        RCC[CSR](RMVF) = 1; // clears all reset-cause flags
    }
#if STM32H7
    return cause & (1<<26) ? -1 :     // iwdg
           cause & (5<<21) ? 2 :      // por/bor
           cause & (1<<17) ? 1 : 0;   // nrst, or other
#else
    return cause & (1<<29) ? -1 :     // iwdg
           cause & (1<<27) ? 2 :      // por/bor
           cause & (1<<26) ? 1 : 0;   // nrst, or other
#endif
}

#if STM32H7 && !STM32H743xx // only STM32H745xx needs this workaround
#define IWDG IWDG1
#endif

void init (int rate) {
    while (IWDG[SR](0)) {}  // wait until !PVU
    IWDG[KR] = 0x5555;      // unlock PR
    IWDG[PR] = rate;        // max timeout, 0 = 400ms, 7 = 26s
    IWDG[KR] = 0xCCCC;      // start watchdog
    reload();
}

void reload (int n) {
    kick();
    while (IWDG[SR](0)) {}  // wait until !PVU
    IWDG[KR] = 0x5555;      // unlock PR
    IWDG[RLR] = n;
    kick();
}

void kick () {
    IWDG[KR] = 0xAAAA;      // reset the watchdog timout
}

} // namespace jeeh::dog

// cache management code needs the CMSIS headers

#if STM32F7
#include <stm32f7xx.h>
#elif STM32H7 && !CORE_CM4
#include <stm32h7xx.h>
#endif

namespace jeeh::cache {

#if STM32F7 || (STM32H7 && !CORE_CM4)

void enable () {
    SCB_EnableICache();
    SCB_EnableDCache();
}

void disable () {
    SCB_DisableICache();
    SCB_DisableDCache();
}

void invalCode () {
    SCB_InvalidateICache();
}

void clean (void const* ptr, uint32_t len) {
    SCB_CleanDCache_by_Addr((uint32_t*) ptr, len);
}

void inval (void const* ptr, uint32_t len) {
    SCB_InvalidateDCache_by_Addr((void*) ptr, len);
}

void flush (void const* ptr, uint32_t len) {
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*) ptr, len);
}

#endif // STM32F7 | STM32H7

} // namespace jeeh::cache

#endif // STM32
