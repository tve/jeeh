#include "jee.h"
using namespace jeeh;
#include <cstdarg>
#include <cstdio>

inline namespace {

    int irqState () {
        switch (SCB[0x4] & 0x1FF) {
            case 0:           return -1; // thread mode, not in any exception
            case 11: case 14: return 0;  // currently in SVC or PendSV
            default:          return 1;  // in some other interrupt
        }
    }

} // inline namespace

//----------------------------------------------------------------------- logf

void jeeh::logf (char const* fmt ...) {
    constexpr IoReg<0xE000'0000> ITM;
    enum { TER=0xE00, TCR=0xE80 };

    // check for enabled ITM flags before generating any printf output
    if (ITM[TCR](0) && ITM[TER](0)) {
        static char buf [80];

        va_list ap;
        va_start(ap, fmt);
        auto n = vsnprintf(buf, sizeof buf, fmt, ap);
        va_end(ap);

        if (n >= (int) sizeof buf)
            n = sizeof buf;
        else if (n == 0 || buf[n-1] != '\n')
            ++n;
        buf[n-1] = '\n';

        itmWrite(buf, n);
    }
}

//---------------------------------------------------------------------- Chain

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

//----------------------------------------------------------------------- Task

inline namespace {

    Task* tasks [Task::LIMIT];
    uint8_t current;

    Task& currTask () {
        auto tp = tasks[current];
        assert(tp != nullptr);
        return *tp;
    }

} // inline namespace

Task::Task () : Message {} {
    static_assert(LIMIT < (int) Device::BASE); // must not overlap device id's

    for (auto i = 0; i < LIMIT; ++i)
        if (tasks[i] == nullptr) {
            tId = i;
            tasks[i] = this;
            return;
        }
    fail(); // too many tasks
}

void Task::submit (Message& msg) {
    assert(irqState() == 0); // must be in either SVC or PendSV
    process(msg); // TODO return value >0 must start the task's timer
}

Task& Task::byId (uint8_t id) {
    assert(id < Task::LIMIT);
    assert(tasks[id] != nullptr);
    return *tasks[id];
}

//--------------------------------------------------------------------- Thread

struct Thread : Task {
    Thread () {
    }

    int process (Message& msg) override {
        append(msg); // TODO ...
        return 0; // TODO
    }
};

inline namespace {

    Thread mainThread; // this self-installs as task #0 TODO yuck ...

} // inline namespace

//--------------------------------------------------------------------- Device

inline namespace {

    uint32_t pending;
    Device* devices [Device::LAST-Device::BASE+1];
    uint8_t interrupts [(uint8_t) Irq::limit];

    void triggerPendSV () {
        SCB[0x04](28) = 1;     // ICSR PENDSVSET
    }

    uint32_t processTriggers () {
        assert(irqState() == 0); // must be in PendSV
        auto p = __atomic_exchange_n(&pending, 0, __ATOMIC_RELAXED);
        while (p != 0) {
            auto i = __builtin_ctz(p); // gcc can count trailing zeros
            assert(devices[i] != nullptr);
            devices[i]->finish();
            p &= ~(1<<i);
        }
        // TODO to switch contexts, return a ptr to {&oldsp,newsp} struct
        return 0;
    }

} // inline namespace

Device::Device (uint8_t id) : dId (id) {
    static_assert(LAST < BASE + 32); // bitmap must fit in uint32_t

    assert(BASE <= id && id <= LAST);
    auto x = id - BASE;
    assert(devices[x] == nullptr);
    devices[x] = this;
}

void Device::irqInstall (uint8_t num, uint8_t prio) {
    // adjust priorities before they might interfere with "real" IRQs
    SCB.byte(0x1F) = 0xDF; // irq #11: SVC
    SCB.byte(0x22) = 0xFF; // irq #14: PendSV
//  SCB.byte(0x23) = 0xFF; // irq #15: SysTick - now in Ticker::init

    assert(num < (uint8_t) Irq::limit);
    interrupts[num] = dId;
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

void Device::reply (Message* mp) {
    assert(irqState() == 0); // must be in either SVC or PendSV
    if (mp != nullptr) {
        auto id = mp->mDst;
        mp->mDst = dId; // restore original destination, i.e. this driver
        Task::byId(id).submit(*mp);
    }
}

//------------------------------------------------------------------ send/recv

void sys::send (Message& m) {
    auto f = +[](Message& msg) {
        auto id = msg.mDst;
        msg.mDst = current;
        if (id < Task::LIMIT)
            Task::byId(id).submit(msg);
        else
            Device::byId(id).start(msg);
    };
    svc((int) f, (int) &m);
}

Message& sys::recv () {
    auto f = +[]() {
        auto& ct = currTask();
        if (ct.isEmpty()) {
            SCB[0x10](4) = 1; // SEVONPEND to wake even when irqs are disabled
            asm ("wfe");      // make sure "real" IRQs will resume after this
        }
        return ct.pull();
    };
    while (true)
        if (auto mp = (Message*) svc((int) f); mp != nullptr)
            return *mp;
}

void sys::call (Message& msg) {
    send(msg);
    (void) recv();
}

//----------------------------------------------------------------------- pool

uint8_t* sys::pool (uint32_t b, uint8_t* p, uint32_t a) {
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

//------------------------------------------------------------------ HardFault

extern "C" [[gnu::naked]]
void HardFault_Handler () {
    asm volatile (
#if STM32L0
        " mov   r0,lr  \n"
        " mov   r1,#4  \n"
        " tst   r0,r1  \n"
        " bne   1f     \n"
        " mrs   r0,msp \n"
        " b     2f     \n"
        "1:            \n"
        " mrs   r0,psp \n"
        "2:            \n"
        " bx    %0     \n"
    :: "r" (hardFaulter)
#else
        " tst   lr,#4  \n"
        " ite   eq     \n"
        " mrseq r0,msp \n"
        " mrsne r0,psp \n"
        " bx    %0     \n"
    :: "r" (hardFaulter)
#endif
    );
}

//--------------------------------------------------------------------- PendSV

extern "C" [[gnu::naked]]
void PendSV_Handler () {
    asm (
        " push     {r0,lr}       \n"
        " blx      %0            \n"
        " cmp      r0,#0         \n"
        " beq      1f            \n"
        " ldmia    r0,{r1,r2}    \n"

        " mrs      r0,psp        \n"
#if FPU_USED
        " tst      lr,#0x10      \n"
        " it       eq            \n"
        " vstmdbeq r0!,{s16-s31} \n"
#endif
        " stmdb    r0!,{r4-r11}  \n"

        " str      r0,[r1]       \n"

        " ldmia    r2!,{r4-r11}  \n"
#if FPU_USED
        " tst      lr,#0x10      \n"
        " it       eq            \n"
        " vldmiaeq r2!,{s16-s31} \n"
#endif
        " msr      psp,r2        \n"
        " bx       lr            \n"

        "1:                      \n"
        " pop      {r0,pc}       \n"
    :: "r" (processTriggers));
}

//------------------------------------------------------------------------ SVC

[[gnu::naked, gnu::noinline]]
int sys::svc (int, int, int, int) {
    asm ("svc 0; bx lr");
}

extern "C" [[gnu::naked]]
void SVC_Handler () {
    asm (
#if STM32L0
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

        " pop   {r1,lr}     \n"
        " str   r0,[r1]     \n"
        " bx    lr          \n"
    );
}
