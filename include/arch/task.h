#define IRQ_HANDLER(name, func) \
    extern "C" void name##_IRQHandler () { func(); }

#ifndef MAX_EVENTS
#define MAX_EVENTS 50
#endif

struct Event {
    uint32_t eDst :8;
    uint32_t eTag :8;
    uint32_t eVal :16;

    Event (uint8_t dst =0, uint8_t tag =0, uint16_t val =0)
        : eDst (dst), eTag (tag), eVal (val) {}

    operator bool () const { return eDst != 0; }
};

struct EventList {
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

        // safely insert the event in this task's chain
        auto next = head;
        do
            wPending[slot] = Event (next, evt.eTag, evt.eVal);
        while (!safeSetIfMatch(head, next, slot));
    }

    Event pull (uint8_t dst) {
        assert(dst > 0);

        // safely remove the first event from this task's chain
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

struct Task {
    constexpr static auto MAX_taskS = 20, MAX_HISTORY = 16;
    enum STATS { S_SEND, S_DELAY, S_PREEMPT, S_REPLY };
    enum HISTS { H_SEND, H_REPLY, H_IRQ, H_PULL };

    static inline uint8_t level; // index of currently active task

    Task (char const* name =nullptr) { setName(name); }
    ~Task () { tasks[tId] = nullptr; }

    uint8_t init () {
        if (tId == 0) {
            tId = MAX_taskS; // assign id's in decreasing order
            while (tasks[--tId] != nullptr)
                assert(tId > 0);
            tasks[tId] = this;
        }
        send({ tId, 0 }); // send START event (always zero)
        return tId;
    }

    static void send (Event evt, Event done ={}) {
        assert(irqState() == 0); // may not be called from an IRQ handler
        assert(evt.eDst > level);
        saveInHist(H_SEND, evt);
        dispatch(evt.eDst, evt, done);
    }

    static void irqPendSV () {
        assert(irqState() == 0);         // PendSV magic ...
        dispatch(MAX_taskS-1, {}, {}); // TODO worst case
    }

#if NOSTATS && !HIST_BASE
    void setName (char const*) {}
#else
    char const* tName;

    void setName (char const* name) { tName = name; }
#endif

#if NOSTATS
    void showStats () {}
#else
    uint32_t tStats [4] ={};

    static void showStats () {
        logf("%20s %9s %9s %9s %9s",
                "TASK", "SEND", "DELAY", "PREEMPT", "REPLY");
        for (auto i = 0; i < MAX_taskS; ++i) {
            auto w = tasks[i];
            if (w != nullptr)
                logf("%15s #%3d %9u %9u %9u %9u",
                    w->tName != nullptr ? w->tName : "", i,
                    w->tStats[S_SEND],
                    w->tStats[S_DELAY],
                    w->tStats[S_PREEMPT],
                    w->tStats[S_REPLY]);
        }
    }
#endif // NOSTATS

#if HIST_BASE
    static inline Event* history;
    static inline uint8_t histPos;

    static void showHistory () {
        Event histBuf [MAX_HISTORY];
        memcpy(histBuf, (Event*) HIST_BASE, sizeof histBuf);

        // find first entry not preceded by an empty entry
        auto first = 0;
        for (auto i = 1; i < MAX_HISTORY; ++i)
            if (histBuf[i] && !histBuf[i-1])
                first = i;
        // show entries, wrap around at end
        for (auto i = 0; i < MAX_HISTORY; ++i) {
            auto evt = histBuf[(first+i) % MAX_HISTORY];
            if (!evt)
                break;
            auto id = evt.eDst & 0x3F;
            auto name = id < MAX_taskS && tasks[id] != nullptr ?
                                tasks[id]->tName : "";
            logf("%4d: [%c] dst %-3d tag %-3d val %-5d %s", 
                    i+1, "SRIP"[evt.eDst>>6], id, evt.eTag, evt.eVal, name);
        }

        // enable history logging once shown
        if (history == nullptr) {
            memset((Event*) HIST_BASE, 0, MAX_HISTORY * sizeof (Event));
            history = (Event*) HIST_BASE;
        }
    }
#else
    static void showHistory () {}
#endif // HIST_BASE

    static void irqClear (Irq irq) {
        auto num = (uint16_t) irq;
        NVIC[0x180 + 4*(num/32)] = 1 << num % 32;
    }

    uint8_t tId =0; // index (and priority) of this task
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
        Event evt { tId, tag, val };
        saveInHist(H_IRQ, evt);
        wPend.push(evt);
        if (tId > level) {
            SCB[0x04](28) = 1; // ICSR PENDSVSET
            stats(S_PREEMPT);
        }
    }

    static void reply (Event evt) {
        auto dst = evt.eDst;
        if (dst != 0) {
            auto w = tasks[dst];
            assert(dst <= level && w != nullptr);
            w->stats(S_REPLY);
            saveInHist(H_REPLY, evt);
            w->wPend.push(evt);
        }
    }

private:
    EventList wPend;  // pending events

    static inline Task* tasks [MAX_taskS];

#if NOSTATS
    void stats (STATS) {}
#else
    void stats (STATS s) { ++tStats[s]; }
#endif

    static void saveInHist (HISTS type, Event evt) {
#if HIST_BASE
        if (history != nullptr) {
            evt.eDst |= type << 6;
            history[histPos] = evt;
            histPos = (histPos+1) % MAX_HISTORY;
            history[histPos] = {}; // clear next entry to mark the end
        }
#else
        (void) type, (void) evt;
#endif
    }

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
            auto w = tasks[level];
            assert(w != nullptr);
            w->stats(S_SEND);
            reply(w->process(evt, done));
        }
        while (level > prev && tasks[level] != nullptr) {
            tasks[level]->unpend();
            --level;
        }
        level = prev;
    }

    void unpend () {
        assert(tId != 0);
        auto evt = wPend.pull(tId);
        if (evt.eDst != 0) {
            unpend(); // use recursion to process in FIFO iso LIFO order
            stats(S_DELAY);
            saveInHist(H_PULL, evt);
            reply(process(evt, {}));
        }
    }
};
