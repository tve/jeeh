#if STM32F1
#define SYSCFG      AFIO
#elif STM32L0 && !STM32L073xx
#define SYSCFG      SYSCFG_COMP
#endif

#if STM32F3
#define EXTI2       EXTI2_TSC
#endif

namespace jeeh {

struct ExtIrq : Device, Chain {
    enum { NONE, RISE, FALL, BOTH };

    ExtIrq () : Device ('E') {
#if !(STM32L0 | STM32WL)
        RCC(ena::SYSCFG, 1) = 1;
#endif
#if STM32G0 | STM32L0
        irqInstall((int) Irq::EXTI0_1);
        irqInstall((int) Irq::EXTI2_3);
        irqInstall((int) Irq::EXTI4_15);
#else
        irqInstall((int) Irq::EXTI0);
        irqInstall((int) Irq::EXTI1);
        irqInstall((int) Irq::EXTI2);
        irqInstall((int) Irq::EXTI3);
        irqInstall((int) Irq::EXTI4);
        irqInstall((int) Irq::EXTI9_5);
        irqInstall((int) Irq::EXTI15_10);
#endif
    }

    void start (Message& m) override {
        append(m);

        auto pos = m.mLen;
        auto off = 4*(pos%4) + 32*(pos/4);
        SYSCFG[EXTICR1](off, 4) = m.mTag - 'A';

        auto mode = (int) m.mPtr; // NONE / RISE / FALL / BOTH
        if (mode != 0) {
            EXTI[RTSR](pos) = (mode & RISE) != 0;
            EXTI[FTSR](pos) = (mode & FALL) != 0;
            EXTI[IMR](pos) = 1;
        } else
            EXTI[IMR](pos) = 0;
    }

    void cancel (Message& m) override {
        if (remove(m))
            EXTI[IMR](m.mLen) = 0;
    }

    void finish () override {
        auto f = __atomic_exchange_4(&flags, 0, __ATOMIC_RELAXED);
        auto pp = &cHead;
        while (*pp != nullptr)
            if (f & (1 << (*pp)->mLen)) {
                auto r = *pp;
                *pp = r->mLnk;
                r->mLnk = r;
                reply(r);
            } else
                pp = &(*pp)->mLnk;
    }

    bool interrupt (int) override {
        flags |= EXTI[PR];
        EXTI[PR] = flags; // clear
        return flags != 0;
    }

private:
    uint32_t flags =0;

#if STM32H7 | STM32WL
    enum { EXTICR1=0x08, RTSR=0x00, FTSR=0x04, IMR=0x80, PR=0x0C }; // cpu1
#else
    enum { EXTICR1=0x08, IMR=0x00, RTSR=0x08, FTSR=0x0C, PR=0x14 };
#endif
};

} // namespace jeeh
