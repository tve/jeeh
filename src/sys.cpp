#include "jee.h"
using namespace jeeh;
#include <cstdarg>
#include <cstdio>

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

[[gnu::naked, gnu::noinline]]
int sys::svc (int, int, int, int) {
    asm ("svc 0; bx lr");
}

extern "C" [[gnu::naked]]
void SVC_Handler () {
    asm (
#if STM32L0
        " mov    r0,lr       \n"
        " mov    r1,#4       \n"
        " tst    r0,r1       \n"
        " bne    1f          \n"
        " mrs    r0,msp      \n"
        " b      2f          \n"
        "1:                  \n"
        " mrs    r0,psp      \n"
        "2:                  \n"
#else
        " tst    lr,#4       \n"
        " ite    eq          \n"
        " mrseq  r0,msp      \n"
        " mrsne  r0,psp      \n"
#endif
        " push   {r0, lr}    \n"

        " ldr    r3,[r0]     \n"
        " ldr    r2,[r0,#12] \n"
        " ldr    r1,[r0,#8]  \n"
        " ldr    r0,[r0,#4]  \n"
        " blx    r3          \n"

        " pop    {r1, lr}    \n"
        " str    r0,[r1]     \n"
        " bx     lr          \n"
    );
}
