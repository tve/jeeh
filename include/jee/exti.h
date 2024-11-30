namespace jeeh {

#if STM32F1
#define SYSCFG      AFIO
#elif STM32L0 && !STM32L073xx
#define SYSCFG      SYSCFG_COMP
#endif

#if STM32F3
#define EXTI2       EXTI2_TSC
#endif

struct ExtIrq : Task {
    enum TAG { START, FIRED };
    enum MODE { NONE, RISE, FALL, BOTH };

    ExtIrq () : Task ("exti") {}

    uint8_t init () {
#if !(STM32L0 | STM32WL)
        RCC(ena::SYSCFG,1) = 1;
#endif
#if STM32G0 | STM32L0
        irqEnable(Irq::EXTI0_1);
        irqEnable(Irq::EXTI2_3);
        irqEnable(Irq::EXTI4_15);
#else
        irqEnable(Irq::EXTI0);
        irqEnable(Irq::EXTI1);
        irqEnable(Irq::EXTI2);
        irqEnable(Irq::EXTI3);
        irqEnable(Irq::EXTI4);
        irqEnable(Irq::EXTI9_5);
        irqEnable(Irq::EXTI15_10);
#endif
        return Task::init();
    }

    void irqExti () {
        uint16_t pr = EXTI[PR];
        EXTI[PR] = pr; // clear
        cycle = cycles::count();
        trigger(FIRED, pr);
    }

    void enable (Pin pin, MODE mode, uint8_t tag, uint16_t val =0) {
        auto pos = pin.pin();
        auto off = 4*(pos%4) + 32*(pos/4);
#if STM32G0 // FIXME
        (void) off;
#else
        SYSCFG[EXTICR1](off,4) = pin.port();
#endif

        if (mode != NONE) {
            events[pos] = { level, tag, val };
            EXTI[RTSR](pos) = (mode & RISE) != 0;
            EXTI[FTSR](pos) = (mode & FALL) != 0;
            EXTI[IMR](pos) = 1;
        } else {
            events[pos] = {};
            EXTI[IMR](pos) = 0;
        }
    }

    void disable (Pin pin) {
        enable(pin, NONE, 0);
    }

private:
    Event events [16];
    uint16_t cycle;

    Event process (Event in, Event) override {
        switch (in.eTag) {
            case START:
                break;
            case FIRED:
                // count leading zeros as quick way to iterate through "1" bits
                for (uint32_t v = in.eVal; v != 0; ) {
                    auto i = 31 - __builtin_clz(v); // find next bit
                    v ^= 1 << i;                    // and clear it
                    auto evt = events[i];
                    evt.eVal = cycle;
                    reply(evt);
                }
                break;
            default:
                fail();
        }
        return {};
    }

#if STM32H7 | STM32WL
    enum { EXTICR1=0x08, RTSR=0x00, FTSR=0x04, IMR=0x80, PR=0x0C }; // cpu1
#else
    enum { EXTICR1=0x08, IMR=0x00, RTSR=0x08, FTSR=0x0C, PR=0x14 };
#endif
};

#if STM32G0 | STM32L0
#define EXTIRQ_TRIGGER(name) \
    IRQ_HANDLER(EXTI0_1,   name.irqExti) \
    IRQ_HANDLER(EXTI2_3,   name.irqExti) \
    IRQ_HANDLER(EXTI4_15,  name.irqExti)
#else
#define EXTIRQ_TRIGGER(name) \
    IRQ_HANDLER(EXTI0,     name.irqExti) \
    IRQ_HANDLER(EXTI1,     name.irqExti) \
    IRQ_HANDLER(EXTI2,     name.irqExti) \
    IRQ_HANDLER(EXTI3,     name.irqExti) \
    IRQ_HANDLER(EXTI4,     name.irqExti) \
    IRQ_HANDLER(EXTI9_5,   name.irqExti) \
    IRQ_HANDLER(EXTI15_10, name.irqExti)
#endif

} // namespace jeeh
