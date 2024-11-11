#include "jee.h"
using namespace jeeh;
#include <cstdarg>
#include <cstdio>

// defined in stm32.cpp
extern int nextTick ();

inline namespace {

    [[maybe_unused]]
    int irqState () {
        switch (SCB[0x4] & 0x1FF) {
            case 0:           return -1; // thread mode, not in any exception
            case 11: case 14: return 0;  // currently in SVC or PendSV
            default:          return 1;  // in some other interrupt
        }
    }

    char logBuf [80];

} // inline namespace

//----------------------------------------------------------------------- flags

#if !NOFLAGS

uint32_t jeeh::flagsAtoZ [26]; // A..Z: settings for global use

[[gnu::weak]] uint32_t jeeh::flag (char const* match) {
    assert('A' <= *match && *match <= 'Z');
    auto f = flagsAtoZ[*match++ - 'A'];
    if (*match == 0)
        return f;
    while ('a' <= *match && *match <= 'z')
        if ((f >> (*match++ - 'a')) & 1)
            return 1;
    return 0;
}

#endif

//------------------------------------------------------------------------ logf

[[gnu::weak]] void jeeh::logWriter (void const* ptr, size_t len) {
    swoWrite(ptr, len);
}

void jeeh::logf (char const* fmt ...) {
#if !(STM32G0 | STM32L0) // Cortex M0+ doesn't support ITM
    constexpr IoReg<0xE000'0000> ITM;
    enum { TER=0xE00, TCR=0xE80 };

    // check for enabled ITM flags before generating any printf output
    if (logWriter != swoWrite || (ITM[TCR](0) && ITM[TER](0)))
#endif
    {

        va_list ap;
        va_start(ap, fmt);
        auto n = vsnprintf(logBuf, sizeof logBuf, fmt, ap);
        va_end(ap);

        if (n >= (int) sizeof logBuf)
            n = sizeof logBuf;
        else if (n == 0 || logBuf[n-1] != '\n')
            ++n;
        logBuf[n-1] = '\n';

        logWriter(logBuf, n);
    }
}

//------------------------------------------------------------------------ fail

[[gnu::weak]] void jeeh::fail (void const* a, char const* f, int n) {
    logf("\n" "failed at %s:%d\n"
              "failed caller: %p", f, n, a);
    BlockIRQ irq;
    while (true) {}
}

//------------------------------------------------------------ hardFaultHandler

[[gnu::weak]] void jeeh::hardFaultHandler (uint32_t* sp) {
    enum { CFSR=0x28, HFSR=0x2C, MMAR=0x34, BFAR=0x38 };

    uint32_t hfsr = SCB[HFSR], cfsr = SCB[CFSR],
            bfar = SCB[BFAR], mmar = SCB[MMAR];

    asm ("cpsid i"); // disable all interrupts

    logf("\n[Hard Fault]  SP=%08x  HFSR=%08x  CFSR=%08x", sp, hfsr, cfsr);
    if (hfsr & (1<<30)) {
        if (cfsr & 0xFFFF0000)
            logf("  Usage fault %04x", cfsr >> 16);
        if (cfsr & 0xFF00) {
            logf("  Bus fault %02x", (uint8_t) (cfsr >> 8));
            if (cfsr & (1<<15))
                logf("    BFAR %08x", bfar);
        }
        if (cfsr & 0xFF) {
            logf("  Memory fault %02x", (uint8_t) cfsr);
            if (cfsr & (1<<7))
                logf("    MMAR %08x", mmar);
        }
    }

    logf("\t R0=%08x  R1=%08x  R2=%08x  R3=%08x",
            sp[0], sp[1], sp[2], sp[3]);
    logf("\tR12=%08x  LR=%08x  PC=%08x PSR=%08x",
            sp[4], sp[5], sp[6], sp[7]);

    fail();
}

//------------------------------------------------------------------- HardFault

extern "C" [[gnu::naked]]
void HardFault_Handler () {
    asm volatile (
#if STM32G0 | STM32L0
        " mov   r0,lr  \n"
        " mov   r1,#4  \n"
        " tst   r0,r1  \n"
        " bne   1f     \n"
        " mrs   r0,msp \n"
        " b     2f     \n"
        "1:            \n"
        " mrs   r0,psp \n"
        "2:            \n"
#else
        " tst   lr,#4  \n"
        " ite   eq     \n"
        " mrseq r0,msp \n"
        " mrsne r0,psp \n"
#endif
        " bx    %0     \n"
    :: "r" (hardFaultHandler));
}

//--------------------------------------------------------------------- logDump

void jeeh::logDump (void const* p, int n, char const* msg) {
    if (msg != nullptr)
        logf("%s: (%db)", msg, n);
    auto q = (uint8_t const*) p;
    auto same = false;
    for (int off = 0; off < n; off += 16) {
        if (off > 0 && memcmp(q + off, q + off-16, 16) == 0) {
            if (!same)
                logf("*");
            same = true;
            continue;
        }
        same = false;
        auto p = logBuf;
        p += snprintf(p, sizeof logBuf, " %03x:", off);
        for (int i = 0; i < 16; ++i) {
            if (i % 4 == 0)
                *p++ = ' ';
            if (off+i >= n) {
                *p++ = ' ';
                *p++ = ' ';
            } else
                p += snprintf(p, sizeof logBuf, "%02x", q[off+i]);
        }
        for (int i = 0; i < 16; ++i) {
            if (i % 4 == 0)
                *p++ = ' ';
            auto b = q[off+i];
            *p++ = off+i >= n ? ' ' : ' ' <= b && b <= '~' ? b : '.';
        }
        *p++ = '\n';
        logWriter(logBuf, p - logBuf);
    }
}

//---------------------------------------------------------------------- PendSV

extern "C" [[gnu::naked]]
void PendSV_Handler () {
    asm (
        " mrs r0,psr \n"
        " push {r0,lr} \n"
        " sub sp,#32 \n"
#if STM32G0 | STM32L0
        " mov r0,pc \n"
        " add r0,#1f-.-2 \n" // yuck
#else
        " addw r0,pc,#1f-.-4 \n" // yuck
#endif
        " str r0,[sp,#24] \n"
        " ldr r0,=0x01000000 \n"
        " str r0,[sp,#28] \n"
        " ldr r0,=0xFFFFFFF9 \n"
        " mov lr,r0 \n"
        " bx lr \n"
        "1: \n"
#if STM32G0 | STM32L0
        " ldr r3,=%0 \n"
        " blx r3 \n"
#else
        " bl %0 \n"
#endif
        " svc 0 \n"
        " b . \n"   // never reached
    :: "i" (Worker::irqPendSV));
}

//------------------------------------------------------------------------- SVC

extern "C" [[gnu::naked]]
void SVC_Handler () {
    asm (
#if STM32G0 | STM32L0
        " add sp,#32 \n"
#else
        " tst lr,#0x10 \n"
        " ite eq \n"
        " addeq sp,#104 \n"
        " addne sp,#32 \n"
#endif
        " pop {r0,r1} \n"
        " msr psr,r0 \n"
        " bx r1 \n"
    );
}
