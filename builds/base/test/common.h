// Common code, used in all tests.

#include <unity.h>
#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

#undef assert
#define assert TEST_ASSERT

// tie printf and logf into Unity's output mechanism

extern "C" int _write (int, char* ptr, int len) {
    for (auto i = 0; i < len; ++i)
        putchar(ptr[i]);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

extern void allTests ();

int main () {
    fastClock();
    cycles::init();

    // adjust priorities before they might interfere with "real" IRQs
    //SCB.byte(0x1F) = 0xFF; // irq #11: SVC
    SCB.byte(0x22) = 0xFF; // irq #14: PendSV

    UNITY_BEGIN();
    allTests();
    return UNITY_END();
}

// End of boilerplate, below is for temporary code, to be moved once ready.

// this needs "-DMYSYSTICK" to disable JeeH's default handlers
#define IRQ_HANDLER(name, func) \
    extern "C" void name##_Handler () { func(); }

struct Event {
    uint32_t eDst :8;
    uint32_t eTag :8;
    uint32_t eVal :16;

    Event (uint8_t dst =0, uint8_t tag =0, uint16_t val =0)
        : eDst (dst), eTag (tag), eVal (val) {}
};

struct EventList {
    constexpr static auto MAX_EVENTS = 100;

    bool isEmpty () const {
        return head == 0;
    }

    void save (Event evt) {
        // find a free event slot
        uint8_t slot = free;
        if (slot == 0) {
            slot = ++last;
            assert(slot < MAX_EVENTS); // fail if too many events are pending
        } else
            free = wPending[slot].eVal;

        // insert the event in this worker's chain
        wPending[slot] = Event (head, evt.eTag, evt.eVal);
        head = slot;
    }

    Event pull (uint8_t dst) {
        assert(!isEmpty());
        auto h = head;
        auto evt = wPending[h];
        head = evt.eDst;
        wPending[h].eVal = free;
        free = h;
        evt.eDst = dst; // clobbered while queued
        return evt;
    }

private:
    uint8_t head =0; // chain of pending events

    static inline Event wPending [MAX_EVENTS];
    static inline uint8_t free; // first unused event slot
    static inline uint8_t last; // last event slot used so far
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
        assert(evt.eDst > level);
        dispatch(evt.eDst, evt, done, arg);
    }

    static void reply (Event evt) {
        auto dst = evt.eDst;
        if (dst != 0) {
            assert(dst <= level && workers[dst] != nullptr);
            workers[dst]->wPend.save(evt);
        }
    }

    static void irqPendSV () {
        assert(irqState() == 0);     // PendSV magic ...
        dispatch(MAX_WORKERS-1, {}); // TODO worst case, loops more often
    }

protected:
    uint8_t wId =0; // index (and priority) of this worker

    virtual Event process (Event in, Event out, void* arg) =0;

    void trigger (uint8_t tag, uint16_t val =0) {
        assert(irqState() != 0); // can only be called from an IRQ handler
        wPend.save({ wId, tag, val });
        if (wId > level)
            SCB[0x04](28) = 1; // ICSR PENDSVSET
    }

private:
    EventList wPend;  // pending events

    static inline Worker* workers [MAX_WORKERS];

    static uint32_t irqState () {
        uint32_t ipsr;
        asm ("mrs %0, ipsr" : "=r" (ipsr));
        return ipsr; // current IRQ, or zero if none
    }

    static void dispatch (uint8_t up, Event evt, Event done ={}, void* arg =nullptr) {
        // this is the only place where the level changes up and down
        auto prev = level;
        level = up;
        if (evt.eDst != 0) {
            assert(evt.eDst == level && workers[level] != nullptr);
            reply(workers[level]->process(evt, done, arg));
        }
        while (level > prev && workers[level] != nullptr) {
            workers[level]->unpend();
            --level;
        }
        level = prev;
    }

    void unpend () {
        // TODO this precesses pending events in FIFO order, is this ok?
        while (!wPend.isEmpty())
            process(wPend.pull(wId), {}, nullptr);
    }
};

extern "C" [[gnu::naked]]
void PendSV_Handler () {
    asm (
        " mrs r0,psr \n"
        " push {r0,lr} \n"
        " sub sp,#32 \n"
        " addw r0,pc,#16 \n"
        " str r0,[sp,#24] \n"
        " ldr r0,=0x01000000 \n"
        " str r0,[sp,#28] \n"
        " ldr r0,=0xFFFFFFF9 \n"
        " mov lr,r0 \n"
        " bx lr \n"
        " bl %0 \n" // target of addw above
        " svc 0 \n"
        " b . \n"   // never reached
    :: "i" (Worker::irqPendSV));
}

extern "C" [[gnu::naked]]
void SVC_Handler () {
    asm (
        " tst lr,#0x10 \n"
        " ite eq \n"
        " addeq sp,#104 \n"
        " addne sp,#32 \n"
        " pop {r0,r1} \n"
        " msr psr,r0 \n"
        " bx r1 \n"
    );
}

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

    void delay (uint16_t ms, uint8_t tag, uint16_t val =0) const {
        send({ wId, DELAY, ms }, { level, tag, val });
    }

    void periodic (uint16_t ms, uint8_t tag, uint16_t val =0) const {
        send({ wId, PERIOD, ms }, { level, tag, val });
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
