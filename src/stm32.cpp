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

void jeeh::systemReset () {
    volatile auto n = SystemCoreClock >> 15;
    while (n > 0) --n; // brief delay to let uart TX finish, etc
    asm volatile ("dsb");
    SCB[0x0C] = (0x5FA<<16) | (1<<2); // SCB AIRCR reset
    asm volatile ("dsb");
    while (true) {}
}

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

//------------------------------------------------------------------------ ITM

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

#endif // Cortex M0+
//--------------------------------------------------------------------- Ticker

uint32_t jeeh::clockChange (uint32_t hz) {
    auto n = hz/1000, o = SystemCoreClock/1000;
    if (n != o) {
        STK[0x4] = (STK[0x4] / o ) * n + 1; // make sure it's not zero
        STK[0x8] = 0;
        SystemCoreClock = hz;
    }
    return hz;
}

#if !STM32F1
namespace jeeh::rtc {

#if STM32G4 | STM32L4 | STM32WL
enum { TR=0x00,DR=0x04,SSR=0x08,ISR=0x0C,PRER=0x10,WUTR=0x14,
        CR=0x18,WPR=0x24,CALR=0x28,SCR=0x5C,BKPR=0x100 };
#else
enum { TR=0x00,DR=0x04,CR=0x08,ISR=0x0C,PRER=0x10,WUTR=0x14,
        WPR=0x24,SSR=0x28,CALR=0x3C,BKPR=0x50 };
#endif
enum { ALRMAR=0x1C, SHIFTR=0x2C, ALRMASSR=0x44 };

#if STM32F3
enum { BDCR=0x20, CSR=0x24 };
#elif STM32F4 | STM32F7 | STM32H7
enum { BDCR=0x70, CSR=0x74 };
#elif STM32L0
enum { CSR=0x50 };
#else
enum { BDCR=0x90, CSR=0x94 };
#endif

#if !STM32L0
void reset () {
    RCC[BDCR](16) = 1; // BDRST
    //sys::wait(2);
    RCC[BDCR](16) = 0; // ~BDRST
}
#endif

void init (bool lse) {
#if !(STM32F3 | STM32F4 | STM32F7 | STM32L0)
    RCC(ena::RTCAPB,1) = 1;
#endif
#if !(STM32H7 | STM32WL)
    RCC(ena::PWR,1) = 1;
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

void deinit () {
#if !STM32L0
    RCC[BDCR](15) = 0; // ~RTCEN
#endif
}

uint8_t fromBcd (uint8_t v) {
    return v - 6 * (v>>4);
}

uint8_t toBcd (uint8_t v) {
    return v + 6 * (v/10);
}

DateTime getDate () {
    uint32_t ssr, tod, doy;
    do { // loop until SSR is stable during all reads
        ssr = RTC[SSR];
        tod = RTC[TR];
        doy = RTC[DR];
    } while ((int) ssr != RTC[SSR]);

    DateTime dt;
    dt.ms = ((uint8_t) ~ssr * 1000) / 256; // assumes PREDIV_S is 255
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
    RTC[ISR](7) = 1; // set INIT
    while (!RTC[ISR](6)) {} // ~INITF
    RTC[TR] = toBcd(dt.ss) | (toBcd(dt.mm) << 8) | (toBcd(dt.hh) << 16);
    RTC[DR] = toBcd(dt.dy) | (toBcd(dt.mo) << 8) | (toBcd(dt.yr) << 16);
#if STM32WL
    RTC[ISR](9) = 1; // BIN 1x, mixed mode
#endif
    RTC[ISR](7) = 0; // clear INIT
}

void set (uint32_t t) {
    set(DateTime (t));
}

void calibrate (int diff) {
    assert(-512 < diff && diff < 512);
    if (diff < 0)
        diff = (1<<15) | (diff+511); // CALP CALM
    RTC[CALR] = diff;
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
    return cause & (1<<26) ? 0 :      // iwdg
           cause & (5<<21) ? 1 :      // por/bor
           cause & (1<<17) ? 2 : -1;  // nrst, or other
#else
    return cause & (1<<29) ? 0 :      // iwdg
           cause & (1<<27) ? 1 :      // por/bor
           cause & (1<<26) ? 2 : -1;  // nrst, or other
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
