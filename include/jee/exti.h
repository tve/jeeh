namespace jeeh {

#if STM32F1
#define SYSCFG      AFIO
#elif STM32L0 && !STM32L073xx
#define SYSCFG      SYSCFG_COMP
#endif

#if STM32F3
#define EXTI2       EXTI2_TSC
#endif

struct ExtIrq : Worker {
    enum TAG { FIRED };
    enum MODE { NONE, RISE, FALL, BOTH };

    ExtIrq () : Worker ("ExtIrq") {}

    uint8_t init () {
#if !(STM32L0 | STM32WL)
        RCC(ena::SYSCFG, 1) = 1;
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
        return Worker::init();
    }

    void irqExti () {
        uint16_t pr = EXTI[PR];
        EXTI[PR] = pr; // clear
        trigger(FIRED, pr);
    }

    void enable (Pin pin, MODE mode, uint8_t tag, uint16_t val =0) {
        auto pos = pin.pin();
        auto off = 4*(pos%4) + 32*(pos/4);
        SYSCFG[EXTICR1](off,4) = pin.port();

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

    Event process (Event in, Event, void*) override {
        switch (in.eTag) {
            case FIRED:
                for (auto i = 0; in.eVal != 0; ++i, in.eVal >>= 1)
                    if (in.eVal & 1)
                        reply(events[i]);
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
#define EXTIRQ_INSTALL(name) \
    IRQ_HANDLER(EXTI0_1,   name.irqExti) \
    IRQ_HANDLER(EXTI2_3,   name.irqExti) \
    IRQ_HANDLER(EXTI4_15,  name.irqExti)
#else
#define EXTIRQ_INSTALL(name) \
    IRQ_HANDLER(EXTI0,     name.irqExti) \
    IRQ_HANDLER(EXTI1,     name.irqExti) \
    IRQ_HANDLER(EXTI2,     name.irqExti) \
    IRQ_HANDLER(EXTI3,     name.irqExti) \
    IRQ_HANDLER(EXTI4,     name.irqExti) \
    IRQ_HANDLER(EXTI9_5,   name.irqExti) \
    IRQ_HANDLER(EXTI15_10, name.irqExti)
#endif

} // namespace jeeh
