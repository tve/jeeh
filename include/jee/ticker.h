namespace jeeh {

struct Ticker : Task {
    constexpr static auto MAX_TIMERS = 20;
    enum TAG { START, TICK, DELAY, PERIOD, CANCEL };

    Ticker () : Task ("tick") {}

    uint8_t init () {
        SCB.byte(0x23) = 0x40; // irq #15: SysTick prio lowered a bit
        setRate(1);
        return Task::init();
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
        send({ tId, DELAY, ms }, { level, tag });
    }

    void periodic (uint16_t ms, uint8_t tag) const {
        assert(ms > 0);
        send({ tId, PERIOD, ms }, { level, tag });
    }

    void cancel (uint8_t tag) const {
        send({ tId, CANCEL, (uint16_t) ((level<<8) | tag) });
    }

    void showInfo () const {
        auto count = 0, repeat = 0;
        for (auto curr = tHead; curr != 0; curr = links[curr]) {
            ++count;
            repeat += period[curr] != 0;
        }
        logf("ticker active %d periodic %d used %d/%d",
                count, repeat, tLast, MAX_TIMERS);
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

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START: // TODO not meaningful, added for consistency
                break;
            case TICK:
                while (expired())
                    timeout();
                // can't return as reply, multiple timers may have expired
                assert(out.eDst == 0);
                break;
#if 0
            case RATE:
                out.eVal = tRate;
                setRate(in.eVal);
                break;
#endif
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
        if (ms == 0) // not periodic
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

#define TICKER_TRIGGER(w) \
    extern "C" void SysTick_Handler () { (w).irqSysTick(); }

} // namespace jeeh
