using jeeh::NVIC;
using jeeh::SCB;
using jeeh::STK;
using jeeh::fail;
using jeeh::Irq;

#define IRQ_HANDLER(name, func) \
    extern "C" void name##_IRQHandler () { func(); }

struct Event {
    uint32_t eDst :8;
    uint32_t eTag :8;
    uint32_t eVal :16;

    Event (uint8_t dst =0, uint8_t tag =0, uint16_t val =0)
        : eDst (dst), eTag (tag), eVal (val) {}
};

struct EventList {
    constexpr static auto MAX_EVENTS = 100;

    void push (Event evt) {
        // safely find a free event slot
        uint8_t slot = free;
        do
            if (slot == 0) {
                slot = safePreInc(last);
                assert(slot < MAX_EVENTS); // too many pending events
                break;
            }
        while (!safeSetIfMatch(free, slot, wPending[slot].eDst));

        // safely insert the event in this worker's chain
        auto next = head;
        do
            wPending[slot] = Event (next, evt.eTag, evt.eVal);
        while (!safeSetIfMatch(head, next, slot));
    }

    Event pull (uint8_t dst) {
        assert(dst > 0);

        // safely remove the first event from this worker's chain
        auto slot = head;
        do
            if (slot == 0)
                return {}; // no more events
        while (!safeSetIfMatch(head, slot, wPending[slot].eDst));

        auto evt = wPending[slot];
        evt.eDst = dst; // clobbered while queued

        // safely return slot to free list
        auto next = free;
        do
            wPending[slot].eDst = next;
        while (!safeSetIfMatch(free, next, slot));

        return evt;
    }

private:
    uint8_t head =0; // chain of pending events

    static inline Event wPending [MAX_EVENTS];
    static inline uint8_t free; // first unused event slot
    static inline uint8_t last; // last event slot used so far

    static uint8_t safePreInc (uint8_t& v) {
        return __atomic_add_fetch(&v, 1, __ATOMIC_RELAXED);
    }

    static bool safeSetIfMatch(uint8_t& dst, uint8_t& exp, uint8_t val) {
        return __atomic_compare_exchange(&dst, &exp, &val, false,
                                          __ATOMIC_RELAXED, __ATOMIC_RELAXED);
    }
};

struct Worker {
    constexpr static auto MAX_WORKERS = 20;

    static inline uint8_t level; // index of currently active worker

    Worker () {}
    ~Worker () { workers[wId] = nullptr; }

    uint8_t init () {
        if (wId == 0) {
            wId = MAX_WORKERS; // assign id's in decreasing order
            while (workers[--wId] != nullptr)
                assert(wId > 0);
            workers[wId] = this;
        }
        return wId;
    }

    static void send (Event evt, Event done ={}, void* arg =nullptr) {
        assert(irqState() == 0); // may not be called from an IRQ handler
        assert(evt.eDst > level);
        dispatch(evt.eDst, evt, done, arg);
    }

    static void irqPendSV () {
        assert(irqState() == 0);                  // PendSV magic ...
        dispatch(MAX_WORKERS-1, {}, {}, nullptr); // TODO worst case
    }

protected:
    uint8_t wId =0; // index (and priority) of this worker

    virtual Event process (Event in, Event out, void* arg) =0;

    static void irqEnable (Irq irq, uint8_t prio =0x80) {
        auto num = (uint16_t) irq;
        NVIC.byte(0x300+num) = prio;
        NVIC[0x00 + 4*(num/32)] = 1 << num % 32;
    }

    void trigger (uint8_t tag, uint16_t val =0) {
        assert(irqState() != 0); // may only be called from an IRQ handler
        wPend.push({ wId, tag, val });
        if (wId > level)
            SCB[0x04](28) = 1; // ICSR PENDSVSET
    }

    static void reply (Event evt) {
        auto dst = evt.eDst;
        if (dst != 0) {
            assert(dst <= level && workers[dst] != nullptr);
            workers[dst]->wPend.push(evt);
        }
    }

private:
    EventList wPend;  // pending events

    static inline Worker* workers [MAX_WORKERS];

    static uint32_t irqState () {
        uint32_t ipsr;
        asm ("mrs %0, ipsr" : "=r" (ipsr));
        return ipsr; // current IRQ, or zero if none
    }

    static void dispatch (uint8_t up, Event evt, Event done, void* arg) {
        // this is the only place where the level changes up and down
        auto prev = level;
        level = up;
        if (evt.eDst == level) {
            assert(workers[level] != nullptr);
            reply(workers[level]->process(evt, done, arg));
        }
        while (level > prev && workers[level] != nullptr) {
            workers[level]->unpend();
            --level;
        }
        level = prev;
    }

    void unpend () {
        auto evt = wPend.pull(wId);
        if (evt.eDst != 0) {
            unpend(); // use recursion to process in FIFO iso LIFO order
            process(evt, {}, nullptr);
        }
    }
};

struct Ticker : Worker {
    constexpr static auto MAX_TIMERS = 20;
    enum TAG { TICK, RATE, DELAY, PERIOD, CANCEL };

    uint8_t init () {
        setRate(100);
        return Worker::init();
    }

    void irqSysTick () {
        ticks += tRate;
        if (expired())
            trigger(TICK);
    }

    uint32_t millis () const {
        // the result has millisecond resolution, even when rate > 1 ms
        while (true) // spinloop, in case ticks changes midway
            if (uint32_t t = ticks, c = STK[0x8]; t == ticks)
                return t + ((STK[0x4]-c) * 8) / (SystemCoreClock/1000);
    }

    void delay (uint16_t ms, uint8_t tag) const {
        send({ wId, DELAY, ms }, { level, tag });
    }

    void periodic (uint16_t ms, uint8_t tag) const {
        send({ wId, PERIOD, ms }, { level, tag });
    }

    void cancel (uint16_t tag) const {
        send({ wId, CANCEL, (uint16_t) ((level<<8) | tag) });
    }

private:
    volatile uint32_t ticks =0;   // adjusted each time SysTick fires
    Event timers [MAX_TIMERS];    // timer pool
    uint16_t period [MAX_TIMERS]; // non-zero if repeating
    uint8_t links [MAX_TIMERS];   // timer chain
    uint8_t tHead =0;             // first timer in chain
    uint8_t tFree =0;             // first unused timer slot
    uint8_t tLast =0;             // last timer slot used so far
    uint8_t tRate =0;             // current SysTick rate in ms

    Event process (Event in, Event out, void*) override {
        switch (in.eTag) {
            case TICK:
                while (expired())
                    timeout();
                // can't return as reply, multiple timers may have expired
                assert(out.eDst == 0);
                break;
            case RATE:
                out.eVal = tRate;
                setRate(in.eVal);
                break;
            case DELAY:
            case PERIOD:
                add(in.eVal, out, in.eTag == PERIOD);
                return {};
            case CANCEL:
                remove(in.eVal >> 8, in.eVal);
                break;
            default:
                fail();
        }
        return out;
    }

    void setRate (uint8_t ms) {
        ticks = millis(); // don't lose the current partial count
        tRate = ms;
        STK[0x4] = (tRate * (SystemCoreClock/1000)) / 8 - 1; // reload value
        STK[0x8] = 0;
        STK[0x0] = tRate > 0 ? 0b011 : 0; // enable, clk/8 mode
    }

    void add (uint16_t ms, Event out, bool repeat) {
        // find a free timer slot
        uint8_t slot = tFree;
        if (slot == 0) {
            slot = ++tLast;
            assert(slot < MAX_TIMERS); // fail if too many timers are active
        } else
            tFree = links[slot];

        // save the timer event with proper deadline
        auto t = ticks;
        timers[slot] = out;
        timers[slot].eVal = t + ms;
        period[slot] = repeat ? ms : 0;

        // locate the position to insert
        auto p = &tHead;
        while (*p != 0 && ms >= (uint16_t) (timers[*p].eVal - t))
            p = &links[*p];

        // insert before the first timer past this one (or at the end)
        links[slot] = *p;
        *p = slot;
    }

    void timeout () {
        auto slot = tHead;
        tHead = links[slot];

        auto evt = timers[slot];
        reply(evt);

        auto ms = period[slot];
        links[slot] = tFree;
        tFree = slot;
        if (ms == 0)
            return;

        ms += ticks - evt.eVal; // correct for missed ticks
        assert(ms <= 60'000);
        add(ms, evt, true); // reschedule
    }

    void remove (uint8_t dst, uint8_t tag) {
        for (auto p = &tHead; *p != 0; p = &links[*p]) {
            auto& t = timers[*p];
            if (t.eDst == dst && t.eTag == tag) {
                auto slot = *p;
                *p = links[*p];
                links[slot] = tFree;
                tFree = slot;
                break;
            }
        }
    }

    bool expired () const {
        return tHead != 0 &&
                (uint16_t) (timers[tHead].eVal - ticks - 1) > 60000;
    }
};

#define TICKER_INSTALL(name) \
    extern "C" void SysTick_Handler () { name.irqSysTick(); }
