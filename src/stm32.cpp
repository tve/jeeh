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
#elif STM32WL
#include "arch/stm32wl.h"
#endif

} // namespace jeeh

using namespace jeeh;

//------------------------------------------------------------------------ SWO

#if !(STM32G0 | STM32L0) // Cortex M0+ doesn't support ITM

void jeeh::swoInit (uint32_t baud, uint32_t hz) {
    constexpr IoReg<0xE000'0000> ITM;
    enum { TER=0xE00, TPR=0xE40, TCR=0xE80, LAR=0xFB0 };

    constexpr IoReg<0xE000'1000> DWT;
    enum { CTRL=0x000 };

    constexpr IoReg<0xE000'EDF0> CoreDebug;
    enum { DEMCR=0x0C };

    constexpr IoReg<0xE0040000> TPI {};
    enum { ACPR=0x010, SPPR=0x0F0, FFCR=0x304 };

#if !STM32H7
    constexpr IoReg<0xE004'2000> DBGMCU;
#endif
    enum { CR=0x04 };

    CoreDebug[DEMCR](24) = 1; // TRCENA
    DBGMCU[CR] = 0x00000027;  // DBGMCU_CR: IOEN STANDBY STOP SLEEP
    TPI[SPPR] = 0x00000002;   // SWO trace output
    TPI[ACPR] = (hz/baud)-1;  // clock prescaler
    ITM[LAR] = 0xC5ACCE55;    // Lock Access
    ITM[TCR] = 0x0001000D;    // Trace Control
    ITM[TPR] = ~0;            // Trace Privilege
    ITM[TER] = ~0;            // Trace Enable
    DWT[CTRL] = 0x4000'03FE;  // Data Watchpoint and Trace
    TPI[FFCR] = 0x0000'0100;  // Formatter and Flush Control
}

#endif

//------------------------------------------------------------------------ ITM

#if !(STM32G0 | STM32L0) // Cortex M0+ doesn't support ITM

void jeeh::swoWrite (void const* ptr, size_t len) {
    constexpr IoReg<0xE000'0000> ITM;
    enum { TER=0xE00, TCR=0xE80 };

    if (ITM[TCR](0) && ITM[TER](0)) { // ITM and channel 0 both enabled
        if (len == 0) // when no args given: flush
            while (ITM[TCR](23)) {} // ~BUSY: wait for ITM to drain

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

#endif

//--------------------------------------------------------------------- Ticker


inline namespace {

struct Ticker : Device, Chain {
    uint16_t rate;
    volatile uint32_t ticks;

    Ticker () : Device (Device::BASE), rate (0), ticks (0) {
        dPower = sys::SHUTDOWN;
        SCB.byte(0x23) = 0xFF; // irq #15: lowest IRQ priority
    }

    // next timeout: -1 if none, 0 if now or overdue, else first timeout ms
    int next () const {
        auto p = first();
        if (p == nullptr)
            return -1;
        uint16_t t = cHead->mLen - ticks;
        return t <= 60'000 ? t : 0;
    }

    void skip (uint16_t ms) {
        STK[0x0] = 0;     // stop the clock, will restart with a new rate
        ticks = millis(); // update actual tick count
        ticks += ms;      // time advances
        finish();         // restart ticker
    }

    void start (Message& msg) override {
        auto ms = msg.mLen;
        assert(ms <= 60'000);

        auto up = next();
        if (up > ms)
            up = ms;          // new entry will become the first one
        if (up < rate) {
            STK[0x0] = 0;     // stop the clock, will restart with a new rate
            ticks = millis(); // update actual tick count
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

    void cancel (Message& msg) override {
        remove(msg);
    }

    void finish () override {
        while (expired())
            reply(pull());

        auto up = next();
        if (up < 0) {
            dPower = sys::SHUTDOWN;
            STK[0x0] = 0; // disable
            return;
        }
        rate = up < 100 ? up : 100;

        // TODO this is a hack: assumes RTC running if DBP bit set in PWR
        //  need SysTick if no RTC
        dPower = PWR[0x00](8) ? sys::STOP2 : sys::SLEEP;

        STK[0x4] = (rate * (SystemCoreClock/1000)) / 8 - 1; // reload value
        STK[0x8] = 0;
        STK[0x0] = 0b011; // enable, clk/8 mode
    }

    bool interrupt (int) override {
        trace(TICK);
        ticks += rate;
        return next() < rate;
    }

    bool expired () const {
        return !isEmpty() && (uint16_t) (cHead->mLen - ticks - 1) > 60'000;
    }

    uint32_t millis () const {
        // the result has millisecond resolution, even when rate > 1
        while (true) // spinloop, in case ticks changes midway
            if (uint32_t t = ticks, c = STK[0x8]; t == ticks) {
                return t + rate - (c*8)/(SystemCoreClock/1000);
            }
    }
};

Ticker ticker;

} // inline namespace

extern "C" void SysTick_Handler () { ticker.irqTrigger(0); }

// TODO these are needed by sys.cpp
int nextTick () { return ticker.next(); }

uint32_t jeeh::clockChange (uint32_t hz) {
    auto n = hz/1000, o = SystemCoreClock/1000;
    if (n != o) {
        STK[0x4] = (STK[0x4] / o ) * n + 1; // make sure it's not zero
        STK[0x8] = 0;
        SystemCoreClock = hz;
    }
    return hz;
}

void sys::wait (uint16_t ms) {
    Message m { ticker.dId, 'T', ms };
    call(m);
}

bool sys::coma (uint32_t sec, int mode) {
    if (ticker.isEmpty())
        return rtc::longSleep(sec, mode);
    uint16_t ms = ticker.next(); // if there is a timeout, don't exceed that
    return rtc::shortSleep(ms < 1000 * sec ? ms : 1000 * sec, mode);
}

#if !STM32F1
namespace jeeh::rtc {

#if STM32G4 | STM32WL
enum { TR=0x00,DR=0x04,SSR=0x08,ISR=0x0C,PRER=0x10,WUTR=0x14,
        CR=0x18,WPR=0x24,SCR=0x5C,BKPR=0x100 };
#else
enum { TR=0x00,DR=0x04,CR=0x08,ISR=0x0C,PRER=0x10,WUTR=0x14,
        WPR=0x24,SSR=0x28,BKPR=0x50 };
#endif
enum { ALRMAR=0x1C, ALRMASSR=0x44 };

#if STM32F3
enum { BDCR=0x20, CSR=0x24 };
#elif STM32F4 | STM32F7 | STM32H7
enum { BDCR=0x70, CSR=0x74 };
#elif STM32L0
enum { CSR=0x50 };
#else
enum { BDCR=0x90, CSR=0x94 };
#endif

#if 0
void reset () {
    RCC[BDCR](16) = 1; // BDRST
    sys::wait(2);
    RCC[BDCR](16) = 0; // ~BDRST
}
#endif

void init (bool lse) {
#if !(STM32F3 | STM32F4 | STM32F7 | STM32L0)
    RCC(ena::RTCAPB, 1) = 1;
#endif
#if !(STM32H7 | STM32WL)
    RCC(ena::PWR, 1) = 1;
#endif
    PWR[0x00](8) = 1; // DBP

    if (lse) {
#if STM32F723xx | STM32WLE5xx
        RCC[BDCR](3,2) = 1;           // LSEDRV on f723d and wl55r
#endif
#if STM32L0
        RCC[CSR](8) = 1;              // LSEON backup domain
        while (RCC[CSR](9) == 0) {}   // wait for LSERDY
        RCC[CSR](16,2) = 1;           // RTSEL = LSE
#else
        RCC[BDCR](0) = 1;             // LSEON backup domain
        while (RCC[BDCR](1) == 0) {}  // wait for LSERDY
        RCC[BDCR](8,2) = 1;           // RTSEL = LSE
#endif
    } else {
        RCC[CSR](0) = 1;              // LSION backup domain
        while (RCC[CSR](1) == 0) {}   // wait for LSIRDY
#if STM32L0
        RCC[CSR](16,2) = 2;           // RTSEL = LSI
#else
        RCC[BDCR](8,2) = 2;           // RTSEL = LSI
#endif
    }
#if STM32L0
    RCC[CSR](18) = 1;                 // RTCEN
#else
    RCC[BDCR](15) = 1;                // RTCEN
#endif

    RTC[WPR] = 0xCA;  // disable write protection, [1] p.803
    RTC[WPR] = 0x53;  // ... and leave it unlocked from now on

    RTC[CR](5) = 1;   // BYPSHAD, this is faster than waiting for RSF

    if (!RTC[ISR](4)) // INITS
        set({0,1,1}); // set to 2000-01-01, so it starts running properly

    SCB[0x10](4) = 1; // SEVONPEND
}

uint8_t fromBcd (uint8_t v) {
    return v - 6 * (v>>4);
}

uint8_t toBcd (uint8_t v) {
    return v + 6 * (v/10);
}

void sleepNow (int mode) {
    assert(mode >= sys::STOP0);
    BlockIRQ irq;
    PWR[0x00](0, 3) = mode - sys::STOP0; // CR1: LPMS
    SCB[0x10](2) = 1; // SLEEPDEEP
    asm ("sev; wfe; wfe");
    SCB[0x10](2) = 0; // ~SLEEPDEEP
}

bool shortSleep (uint16_t ms, int mode) {
    if (ms > 16'000)
        ms = 16'000;
    auto sel = 3;
    auto count = (100'000*ms) / 6104; // 61.035 us, but need to avoid overflow
    while (count >= 32768) {
        --sel;
        count /= 2;
    }
    assert(sel >= 0);
//RTC[SCR] = 1<<2;    // CWUTF

#if STM32WL
    EXTI[0x00](20) = 1; // RT20 in RTSR1
    EXTI[0x84](20) = 1; // EM20 in EMR1
#else
    EXTI[0x08](20) = 1; // RT20 in RTSR1
    EXTI[0x04](20) = 1; // EM20 in EMR1
#endif

    RTC[CR](10) = 0;             // ~WUTE
    while (RTC[ISR](2) == 0) {}  // wait for WUTWF

    RTC[WUTR] = count;
    RTC[CR](0,3) = sel;

    RTC[CR](14) = 1;             // WUTIE
    RTC[CR](10) = 1;             // WUTE

    auto todLast = getDate().todMillis();
    sleepNow(mode);
    ticker.skip(getDate().todMillis() - todLast); // TODO wraparound

    RTC[CR](10) = 0;    // ~WUTE

    bool done = RTC[ISR](10);
#if STM32G4 | STM32WL
    RTC[SCR] = 1<<2;    // CWUTF
#else
    RTC[ISR](10) = 0;   // clear WUTF
#endif
    return done;
}

bool longSleep (uint32_t sec, int mode) {
    assert(sec > 0);
    DateTime dt (getSecs() + sec);

#if STM32WL
    EXTI[0x00](18) = 1; // RT18 in RTSR1
    EXTI[0x84](18) = 1; // EM18 in EMR1
#else
    EXTI[0x08](18) = 1; // RT18 in RTSR1
    EXTI[0x04](18) = 1; // EM18 in EMR1
#endif

    RTC[CR](8) = 0;             // ~ALRAE
#if !STM32WL
    while (RTC[ISR](0) == 0) {} // wait for ALRAWF
#endif

    RTC[ALRMAR] = (toBcd(dt.dy)<<24) | (toBcd(dt.hh)<<16) |
                   (toBcd(dt.mm)<<8) | toBcd(dt.ss);
    //RTC[ALRMASSR] = 0;

    RTC[CR](12) = 1;             // ALRAIE
    RTC[CR](8) = 1;              // ALRAE

    sleepNow(mode);

    if (!RTC[ISR](8))   // ~ALRAF
        return false;
    RTC[CR](8) = 0;     // ~ALRAE: only disable once it has triggered

#if STM32G4 | STM32WL
    RTC[SCR] = 1<<0;    // CALRAF
#else
    RTC[ISR](8) = 0;    // clear ALRAF
#endif
    return true;
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
    dt.ss = fromBcd(tod);
    dt.mm = fromBcd(tod>>8);
    dt.hh = fromBcd((tod>>16) & 0x3F);
    dt.dy = fromBcd(doy);
    dt.mo = fromBcd((doy>>8) & 0x1F);
    // works until end 2063, will fail (i.e. roll over) in 2064 !
    dt.yr = fromBcd(doy>>16);
    return dt;
}

uint32_t getSecs () {
    return getDate(); // let DateTime::operator uint32_t do the conversion
}

void set (DateTime const& dt) {
    RTC[ISR](7) = 1;            // set INIT
    while (RTC[ISR](6) == 0) {} // wait for INITF
    RTC[TR] = toBcd(dt.ss) | (toBcd(dt.mm) << 8) | (toBcd(dt.hh) << 16);
    RTC[DR] = toBcd(dt.dy) | (toBcd(dt.mo) << 8) | (toBcd(dt.yr) << 16);
#if STM32WL
    RTC[ISR](9) = 1;            // BIN 1x, mixed mode
#endif
    RTC[ISR](7) = 0;            // clear INIT
}

void set (uint32_t t) {
    set(DateTime (t));
}

uint32_t getReg (int reg) {
#if STM32G4 | STM32WL
    return TAMP[BKPR+4*reg]; // regs 0..31
#else
    return RTC[BKPR+4*reg];  // regs 0..31
#endif
}

void setReg (int reg, uint32_t val) {
#if STM32G4 | STM32WL
    TAMP[BKPR+4*reg] = val;  // regs 0..31
#else
    RTC[BKPR+4*reg] = val;   // regs 0..31
#endif
}

} // namespace jeeh::rtc
#endif // !STM32F1

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
#elif STM32G4 | STM32G0 | STM32L0 | STM32L4 | STM32WL
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
