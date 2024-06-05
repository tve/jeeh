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

[[gnu::weak]] void jeeh::logWriter (void const* ptr, size_t len) {
    swoWrite(ptr, len);
}

//------------------------------------------------------------------------ logf

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

//--------------------------------------------------------------------- logDump

void jeeh::logDump (void const* p, int n, char const* msg) {
    if (msg != nullptr)
        logf("%s: (%db)", msg, n);
    auto q = (uint8_t const*) p;
    for (int off = 0; off < n; off += 16) {
        if (off > 0 && memcmp(q + off, q + off-16, 16) == 0) {
            if (off > 16 && memcmp(q + off, q + off-32, 16) != 0)
                logf("*");
            continue;
        }
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

//----------------------------------------------------------------------- Chain

bool Chain::insert (Message& msg) {
    assert(!msg.inUse());
    msg.mLnk = cHead;
    cHead = &msg;
    return msg.mLnk == nullptr;
}

bool Chain::append (Message& msg) {
    assert(!msg.inUse());
    auto pp = &cHead;
    while (*pp != nullptr)
        pp = &(*pp)->mLnk;
    msg.mLnk = nullptr;
    *pp = &msg;
    return pp != &cHead;
}

bool Chain::remove (Message& msg) {
    assert(msg.inUse());
    for (auto pp = &cHead; *pp != nullptr; pp = &(*pp)->mLnk)
        if (*pp == &msg) {
            *pp = msg.mLnk;
            msg.mLnk = &msg;
            return true;
        }
    return false;
}

Message* Chain::pull () {
    auto mp = cHead;
    if (mp != nullptr) {
        cHead = mp->mLnk;
        mp->mLnk = mp;
    }
    return mp;
}

//---------------------------------------------------------------------- Thread

inline namespace {
    uint8_t current;   // currently running thread id
    uint8_t nextToRun; // thread id of next thread to run
    bool fixed;        // cannot switch threads when set

    void triggerPendSV () { SCB[0x04](28) = 1; } // ICSR PENDSVSET
}

[[gnu::weak]] uint8_t jeeh::lowestPower (uint8_t power, uint16_t) {
    return power;
}

[[gnu::weak]] void jeeh::resumePower () {}

struct Thread : Message, Chain {
    enum { LIMIT = 20 };
    static_assert(LIMIT < (int) Device::BASE); // must not overlap device id's

    enum { RUN=0x00, WAIT=0x01, DEAD=0x02 };

    uint32_t* sp =nullptr;   // saved stack pointer when not running
    Message* block =nullptr; // block until this specific msg is received
    uint8_t state =RUN;      // current state of this thread

    // see https://en.cppreference.com/w/cpp/memory/new/operator_new
    static void* operator new (size_t, void* p) { return p; }

    Thread () : Message { current } {
        for (auto i = 0; i < LIMIT; ++i)
            if (threads[i] == nullptr) {
                mTag = i;
                threads[i] = this;
                return;
            }
        fail(); // too many threads
    }

    void submit (Message& msg) {
        assert(irqState() == 0); // must be in SVC or PendSV

        if (block == &msg) {
            insert(msg);
            block = nullptr;
        } else
            append(msg);

        if (state == WAIT)
            reschedule();
    }

    static Thread& byId (uint8_t id) {
        auto p = threads[id];
        assert(p != nullptr);
        return *p;
    }

    void reschedule (int newState =RUN) {
        assert(irqState() == 0); // must be in SVC or PendSV
        state = newState;
        if (mTag > nextToRun)
            nextToRun = mTag;

        while (true) {
            auto th = threads[nextToRun];
            assert(th != nullptr);
            if (th->state == RUN) {
                SCB[0x10](1) = 0; // ~SLEEPONEXIT
                if (nextToRun != current)
                    triggerPendSV();
                break;
            }
            if (nextToRun == 0) {
                auto t = nextTick();
                auto power = lowestPower(Device::powerScan(), t);
                if (power >= sys::STOP0)
                    rtc::shortSleep(t, power);
                resumePower();
                SCB[0x10](1) = 1; // SLEEPONEXIT
                break;
            }
            --nextToRun;
        }
    }

    inline static Thread* threads [LIMIT];
};

static Thread mainThread; // this self-installs as thread #0 TODO yuck ...

static Thread& context () {
    return Thread::byId(current);
}

//----------------------------------------------------------------------- Fixer

Fixer::Fixer () : saved (fixed) {
    fixed = true;
}

Fixer::~Fixer () {
    fixed = saved;
    if (nextToRun != current)
        triggerPendSV();
}

//------------------------------------------------------------------------ Lock

bool Lock::acquire (bool blocking) {
    auto r = true;

    Fixer fixer;
    if (!locked)
        locked = true;
    else if (blocking) {
        Message m { current, 'L' };
        waiting.append(m);
        [[maybe_unused]] auto& t = sys::recv(); assert(&t == &m);
        assert(locked);
    } else
        r = false;

    return r;
}

void Lock::release () {
    Fixer fixer;
    assert(locked);
    auto mp = waiting.pull();
    if (mp != nullptr)
        sys::send(*mp);
    else
        locked = false;
}

//---------------------------------------------------------------------- Device

inline namespace {

    uint32_t pending;
    Device* devices [Device::LAST-Device::BASE+1];
    uint8_t interrupts [(uint8_t) Irq::limit];

    void* processTriggers () {
        assert(irqState() == 0); // must be in PendSV
        auto p = __atomic_exchange_n(&pending, 0, __ATOMIC_RELAXED);
        while (p != 0) {
            auto i = __builtin_ctz(p); // gcc can count trailing zeros
            assert(devices[i] != nullptr);
            devices[i]->finish();
            p &= ~(1<<i);
        }
        if (nextToRun == current)
            return nullptr; // no context switch

        // switch contexts: return ptr to {&oldsp,newsp}, see PendSV_Handler
        static struct { uint32_t **oldSp, *newSp; } stacks;
        stacks.oldSp = &context().sp;
        current = nextToRun;
        stacks.newSp = context().sp;
        return &stacks;
    }

} // inline namespace

Device::Device (uint8_t id) : dId (id) {
    static_assert(LAST < BASE + 32); // bitmap must fit in uint32_t

    assert(BASE <= id && id <= LAST);
    auto o = id - BASE;
    assert(devices[o] == nullptr);
    devices[o] = this;
}

void Device::irqInstall (uint8_t num, uint8_t prio) {
    // adjust priorities before they might interfere with "real" IRQs
    SCB.byte(0x1F) = 0xDF; // irq #11: SVC
    SCB.byte(0x22) = 0xFF; // irq #14: PendSV
//  SCB.byte(0x23) = 0xFF; // irq #15: SysTick - now in Ticker::init

    assert(num < (uint8_t) Irq::limit);
    interrupts[num] = dId-BASE;
    NVIC.byte(0x300+num) = prio;
    NVIC[0x00 + 4*(num/32)] = 1 << num % 32;
}

void Device::irqTrigger (uint8_t num) {
    assert(irqState() > 0); // must be in a "real" interrupt
    if (interrupt(num)) {
        __atomic_or_fetch(&pending, 1 << (dId-BASE), __ATOMIC_RELAXED);
        triggerPendSV(); // will call "finish" once back in thread mode
    }
}

Device& Device::byId (uint8_t id) {
    assert(BASE <= id && id <= LAST);
    assert(devices[id-BASE] != nullptr);
    return *devices[id-BASE];
}

uint8_t Device::powerScan () {
    uint8_t min = sys::SHUTDOWN;
    for (auto e : devices)
        if (e != nullptr && min > e->dPower)
            min = e->dPower;
    return min;
}

void Device::reply (Message* mp) {
    assert(irqState() == 0); // must be in either SVC or PendSV
    if (mp != nullptr) {
        auto id = mp->mDst;
        mp->mDst = dId; // restore original destination, i.e. this driver
        Thread::byId(id).submit(*mp);
    }
}

//------------------------------------------------------------------- send/recv

void sys::send (Message& m) {
    assert(irqState() < 0); // must be in thread mode
    auto f = +[](Message& msg) {
        auto id = msg.mDst;
        msg.mDst = context().mTag;
        if (id < Thread::LIMIT)
            Thread::byId(id).submit(msg);
        else
            Device::byId(id).start(msg);
    };
    svc((int) f, (int) &m);
}

Message& sys::recv () {
    assert(irqState() < 0); // must be in thread mode
    auto f = +[]() {
        auto& th = context();
        if (th.block != nullptr || th.isEmpty()) {
            th.reschedule(th.WAIT);
            return (Message*) nullptr;
        }
        return th.pull();
    };
    while (true)
        if (auto mp = (Message*) svc((int) f); mp != nullptr) {
            mp->callback();
            return *mp;
        }
}

void sys::call (Message& msg) {
    auto& th = context();
    th.block = &msg;
    send(msg);
    [[maybe_unused]] auto& r = recv();
    assert(&r == &msg);
    th.block = nullptr;
}

//------------------------------------------------------------------------ pool

uint8_t* sys::pool (uint32_t b, uint8_t* p, uint32_t a) {
    assert(irqState() < 0); // must be in thread mode
    auto f = +[](uint32_t bytes, uint8_t* ptr, uint32_t align) {
        if (bytes > 0) {
            if (align > 8)
                bytes += align - 8; // allocate enough slack
            ptr = (uint8_t*) realloc(ptr, bytes);
            if (align > 8)
                ptr += (1 + ~(uint32_t) ptr) % align;
        } else if (ptr != nullptr) {
            free(ptr);
            ptr = nullptr;
        }
        return ptr;
    };
    return (uint8_t*) svc((int) f, b, (int) p, a);
}

//------------------------------------------------------------------------ init

void sys::init (uint32_t* ptr, uint32_t len) {
    memset(ptr, 0xDD, len * sizeof *ptr);

    // stay in protected thread mode, switch to separate stacks
    asm volatile (
        " mov r3,sp      \n"
        " msr psp,r3     \n"
        " mov r3,#2      \n"
        " msr control,r3 \n"
        " isb            \n"
        " msr msp,%0     \n"
    :: "r" ((uintptr_t) (ptr + len)) : "r3");

#if 0 // see Device::irqInstall
    // PendSV is used to switch stacks with the lowest interrupt priority
    // (SHPR3, PRI_14) it always runs last, i.e. as only active exception
    SCB.byte(0x22) = 0xFF;

    // SVC is used to protect kernel-specific non-preemptible actions
    // (SHPR2, PRI_11) needs to be above PendSV to be callable from it
    // another use is for IRQs which want to be postponed during SVCs
    SCB.byte(0x1F) = 0xDF;
#endif

#if 0
    // TODO can't always switch to unprivileged mode at this point:
    //  - polled console I/O will fail due to access to UART regs
    //  - blinking an LED will need access to GPIO registers
    //  - use of the DWT cycle counter requires privileged mode
    //  - solution: support running some threads as privileged
    asm volatile (
        " mov r1,#3      \n"
        " msr control,r1 \n"
    ::: "r1");
#endif
}

//------------------------------------------------------------------- fork/quit

Message& sys::fork (uint32_t* p, uint16_t n, int (*h)(Message&), intptr_t a) {
    memset(p, 0xDD, n * sizeof *p);
    auto tp = new (p) Thread;
    tp->mDst = current;
    tp->mLen = n;
    tp->mPtr = (uint8_t*) a;

    tp->sp = p + n - 16;
    tp->sp[8] = (uint32_t)(Message*) tp; // r0
    tp->sp[13] = (uint32_t) sys::quit;   // lr
    tp->sp[14] = (uint32_t) h;           // pc
    tp->sp[15] = 0x0100'0000;            // psr

    auto f = +[](Thread* tp) {
        tp->reschedule();
    };
    svc((int) f, (int) tp);
    return *tp;
}

void sys::quit (intptr_t ret) {
    auto& th = context();
    th.mPtr = (uint8_t*) ret;
    send(th);
    th.~Thread();

    auto f = +[](Thread& th) {
        th.reschedule(th.DEAD);
    };
    svc((int) f, (int) &th);
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

//---------------------------------------------------------------------- PendSV

extern "C" [[gnu::naked]]
void PendSV_Handler () {
    asm (
        " push     {r0,lr}       \n"
        " blx      %0            \n"
        " cmp      r0,#0         \n"
        " beq      1f            \n"
        " ldmia    r0!,{r1,r2}    \n"

        " mrs      r0,psp        \n"
#if STM32G0 | STM32L0
        " sub      r0,#32        \n"
        " mov      r3,r0         \n"
        " stmia    r3!,{r4-r7}   \n"
        " mov      r4,r8         \n"
        " mov      r5,r9         \n"
        " mov      r6,r10        \n"
        " mov      r7,r11        \n"
        " stmia    r3!,{r4-r7}   \n"
#else
#if FPU_USED
        " tst      lr,#0x10      \n"
        " it       eq            \n"
        " vstmdbeq r0!,{s16-s31} \n"
#endif
        " stmdb    r0!,{r4-r11}  \n"
#endif

        " str      r0,[r1]       \n"

#if STM32G0 | STM32L0
        " add      r2,#16        \n"
        " ldmia    r2!,{r4-r7}   \n"
        " mov      r8,r4         \n"
        " mov      r9,r5         \n"
        " mov      r10,r6        \n"
        " mov      r11,r7        \n"
        " mov      r1,r2         \n"
        " sub      r1,#32        \n"
        " ldmia    r1!,{r4-r7}   \n"
#else
        " ldmia    r2!,{r4-r11}  \n"
#if FPU_USED
        " tst      lr,#0x10      \n"
        " it       eq            \n"
        " vldmiaeq r2!,{s16-s31} \n"
#endif
#endif
        " msr      psp,r2        \n"
        " bx       lr            \n"

        "1:                      \n"
        " pop      {r0,pc}       \n"
    :: "r" (processTriggers));
}

//------------------------------------------------------------------------- SVC

[[gnu::naked, gnu::noinline]]
int sys::svc (int, int, int, int) {
    asm ("svc 0; bx lr");
}

extern "C" [[gnu::naked]]
void SVC_Handler () {
    asm (
#if STM32G0 | STM32L0
        " mov   r0,lr       \n"
        " mov   r1,#4       \n"
        " tst   r0,r1       \n"
        " bne   1f          \n"
        " mrs   r0,msp      \n"
        " b     2f          \n"
        "1:                 \n"
        " mrs   r0,psp      \n"
        "2:                 \n"
#else
        " tst   lr,#4       \n"
        " ite   eq          \n"
        " mrseq r0,msp      \n"
        " mrsne r0,psp      \n"
#endif
        " push  {r0,lr}     \n"

        " ldr   r3,[r0]     \n"
        " ldr   r2,[r0,#12] \n"
        " ldr   r1,[r0,#8]  \n"
        " ldr   r0,[r0,#4]  \n"
        " blx   r3          \n"

        " pop   {r1,r2}     \n"
        " str   r0,[r1]     \n"
        " bx    r2          \n"
    );
}

//------------------------------------------------------------------------- IRQ

// override all the interrupt handlers to dispatch through a single function
extern "C" {

void irqDispatch () {
    uint8_t irq = SCB[0x4] - 16; // ICSR
    assert(irq < (uint8_t) Irq::limit);
    auto o = interrupts[irq];
    assert(devices[o] != nullptr);
    devices[o]->irqTrigger(irq);
}

// to re-generate "stm32-irqs.h", see the "gen-irqs.sh" script
#if !ARDUINO
#define IRQ(f)      [[gnu::alias ("irqDispatch")]] void f ();
#include "arch/all-irqs.h"
#endif

} // extern "C"
