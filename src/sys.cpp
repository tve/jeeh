#include "jee.h"
using namespace jeeh;
#include <cstdarg>
#include <cstdio>

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

//--------------------------------------------------------------------- Device

uint32_t Device::pending;
Device* Device::devices [LAST-BASE+1];
uint8_t Device::interrupts [(uint8_t) Irq::limit];

Device::Device (uint8_t id) : dId (id) {
    auto x = asIndex(id);
    assert(devices[x] == nullptr);
    devices[x] = this;
}

void Device::irqInstall (uint8_t num, uint8_t prio) {
    // TODO
    //SCB.byte(0x1F) = 0xDF; // SVC
    //SCB.byte(0x22) = 0xFF; // PendSV
    //SCB.byte(0x23) = 0xFF; // SysTick - now in Ticker::init

    assert(num < (uint8_t) Irq::limit);
    interrupts[num] = dId;
    NVIC.byte(0x300+num) = prio;
    NVIC[0x00 + 4*(num/32)] = 1 << num % 32;
}

void Device::irqTrigger (uint8_t num) {
    if (interrupt(num)) {
        __atomic_or_fetch(&pending, 1 << (dId-BASE), __ATOMIC_RELAXED);
#if 0 // TODO
        if (Thread::current != &Thread::dummy)
            triggerPendSV();
#endif
    }
}

Device& Device::byId (uint8_t id) {
    assert(BASE <= id && id <= LAST);
    assert(devices[id-BASE] != nullptr);
    return *devices[id-BASE];
}

void Device::process () {
    auto p = __atomic_exchange_n(&pending, 0, __ATOMIC_RELAXED);
    while (p != 0) {
        auto i = __builtin_ctz(p); // gcc can count trailing zeros
        byId(i+BASE).finish();
        p &= ~(1<<i);
    }
}

void Device::reply (Message* mp) {
    if (mp == nullptr)
        return;
    auto id = mp->mDst;
    mp->mDst = dId; // restore original destination, i.e. this driver
    Task::byId(id).append(*mp);
}

//----------------------------------------------------------------------- Task

Task* tasks [Task::LIMIT];
uint8_t current;
Task mainTask;

Task& currTask () {
    auto tp = tasks[current];
    assert(tp != nullptr);
    return *tp;
}

Task::Task () : Message {} {
    for (auto i = 0; i < Task::LIMIT; ++i)
        if (tasks[i] == nullptr) {
            tid = i;
            tasks[i] = this;
            return;
        }
    fail(); // too many tasks
}

Task& Task::byId (uint8_t id) {
    assert(id < Task::LIMIT);
    assert(tasks[id] != nullptr);
    return *tasks[id];
}

//------------------------------------------------------------------ send/recv

void sys::send (Message& m) {
    assert(&Task::byId(0) == &mainTask);
    auto f = +[](Message& msg) {
        auto id = msg.mDst;
        msg.mDst = current;
        if (Device::BASE <= id && id <= Device::LAST)
            Device::byId(id).start(msg);
        else
            Task::byId(id).append(msg);
    };
    svc((int) f, (int) &m);
}

Message& sys::recv () {
    auto f = +[]() {
        Device::process(); // in case PendSV is not getting called
        auto mp = currTask().pull();
        if (mp == nullptr) {
            SCB[0x10](4) = 1; // SEVONPEND, to wake when irqs are disabled
            asm ("wfe");      // make sure "real" IRQs will resume after this
        }
        return mp;
    };
    while (true) {
        auto mp = (Message*) svc((int) f);
        if (mp != nullptr)
            return *mp;
    }
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
        " push  {r0, lr}    \n"

        " ldr   r3,[r0]     \n"
        " ldr   r2,[r0,#12] \n"
        " ldr   r1,[r0,#8]  \n"
        " ldr   r0,[r0,#4]  \n"
        " blx   r3          \n"

        " pop   {r1, lr}    \n"
        " str   r0,[r1]     \n"
        " bx    lr          \n"
    );
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
