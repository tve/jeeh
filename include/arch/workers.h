#define IRQ_HANDLER(name, func) \
    extern "C" void name##_IRQHandler () { func(); }

struct Event {
    uint32_t eDst :8;
    uint32_t eTag :8;
    uint32_t eVal :16;

    Event (uint8_t dst =0, uint8_t tag =0, uint16_t val =0)
        : eDst (dst), eTag (tag), eVal (val) {}

    operator bool () const { return eDst != 0; }
};

struct EventList {
    constexpr static auto MAX_EVENTS = 100;

    EventList () {
        // adjust priorities before they might interfere with "real" IRQs
        SCB.byte(0x1F) = 0xFF; // irq #11: SVC
        SCB.byte(0x22) = 0xFF; // irq #14: PendSV
    }

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
    enum STATS { S_SEND, S_DELAY, S_PREEMPT, S_REPLY };

    char const* wName;

    static inline uint8_t level; // index of currently active worker

    Worker (char const* name =nullptr) : wName (name) {}
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

    static void send (Event evt, Event done ={}) {
        assert(irqState() == 0); // may not be called from an IRQ handler
        assert(evt.eDst > level);
        dispatch(evt.eDst, evt, done);
    }

    static void irqPendSV () {
        assert(irqState() == 0);         // PendSV magic ...
        dispatch(MAX_WORKERS-1, {}, {}); // TODO worst case
    }

#if NOSTATS
    void showStats () {}
#else
    uint32_t wStats [4] ={};

    static void showStats () {
        logf("%20s %9s %9s %9s %9s",
                "WORKER", "SEND", "DELAY", "PREEMPT", "REPLY");
        for (auto i = 0; i < MAX_WORKERS; ++i) {
            auto w = workers[i];
            if (w != nullptr)
                logf("%15s #%3d %9u %9u %9u %9u",
                    w->wName != nullptr ? w->wName : "", i,
                    w->wStats[S_SEND],
                    w->wStats[S_DELAY],
                    w->wStats[S_PREEMPT],
                    w->wStats[S_REPLY]);
        }
    }
#endif // NOSTATS

    static void irqClear (Irq irq) {
        auto num = (uint16_t) irq;
        NVIC[0x180 + 4*(num/32)] = 1 << num % 32;
    }

    uint8_t wId =0; // index (and priority) of this worker
protected:
    virtual Event process (Event in, Event out) =0;

    static void irqEnable (Irq irq, uint8_t prio =0x80) {
        auto num = (uint16_t) irq;
        NVIC.byte(0x300+num) = prio;
        NVIC[0x000 + 4*(num/32)] = 1 << num % 32;
    }

    static void irqDisable (Irq irq) {
        auto num = (uint16_t) irq;
        NVIC[0x080 + 4*(num/32)] = 1 << num % 32;
    }

    void trigger (uint8_t tag, uint16_t val =0) {
        assert(irqState() != 0); // may only be called from an IRQ handler
        wPend.push({ wId, tag, val });
        if (wId > level) {
            SCB[0x04](28) = 1; // ICSR PENDSVSET
            stats(S_PREEMPT);
        }
    }

    static void reply (Event evt) {
        auto dst = evt.eDst;
        if (dst != 0) {
            auto w = workers[dst];
            assert(dst <= level && w != nullptr);
            w->stats(S_REPLY);
            w->wPend.push(evt);
        }
    }

private:
    EventList wPend;  // pending events

    static inline Worker* workers [MAX_WORKERS];

#if NOSTATS
    void stats (STATS) {}
#else
    void stats (STATS s) { ++wStats[s]; }
#endif

    static uint32_t irqState () {
        uint32_t ipsr;
        asm ("mrs %0, ipsr" : "=r" (ipsr));
        return ipsr; // current IRQ, or zero if none
    }

    static void dispatch (uint8_t up, Event evt, Event done) {
        // this is the only place where the level changes up and down
        auto prev = level;
        level = up;
        if (evt.eDst == level) {
            auto w = workers[level];
            assert(w != nullptr);
            w->stats(S_SEND);
            reply(w->process(evt, done));
        }
        while (level > prev && workers[level] != nullptr) {
            workers[level]->unpend();
            --level;
        }
        level = prev;
    }

    void unpend () {
        assert(wId != 0);
        auto evt = wPend.pull(wId);
        if (evt.eDst != 0) {
            unpend(); // use recursion to process in FIFO iso LIFO order
            stats(S_DELAY);
            process(evt, {});
        }
    }
};
