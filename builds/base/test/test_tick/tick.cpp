// System tick tests.

#include "../common.h"

void setUp () {}
void tearDown () { Worker::resetAll(); }

template< uint8_t MAX >
class Ticker : Worker {
    Event process (Event in, Event out, void*) override {
        switch (in.eTag) {
            case TICK:
                while (expired()) {
                    auto slot = tHead;
                    tHead = links[slot];
                    // can't return as reply, multiple timers may be expiring
                    reply(timers[slot]);
                    links[slot] = tFree;
                    tFree = slot;
                }
                assert(out.eDst == 0);
                break;
            case RATE:
                setRate(in.eVal);
                break;
            case DELAY:
                add(in.eVal, out);
                return {};
            default:
                fail();
        }
        return out;
    }

    void setRate (uint8_t ms) {
        tRate = ms;
        STK[0x4] = (tRate * (SystemCoreClock/1000)) / 8 - 1; // reload value
        STK[0x8] = 0;
        STK[0x0] = 0b011; // enable, clk/8 mode
    }

    void add (uint16_t ms, Event out) {
        // find a free timer slot
        uint8_t slot = tFree;
        if (slot == 0) {
            slot = ++tLast;
            assert(slot < MAX);
        } else
            tFree = links[slot];

        // save the timer event with proper deadline
        auto t = ticks;
        timers[slot] = out;
        timers[slot].eVal = t + ms;

        // locate the position to insert
        auto p = &tHead; // insert in proper position
        while (*p != 0 && ms >= (uint16_t) (timers[*p].eVal - t))
            p = &links[*p];

        // insert before the first timer past this one (or at the end)
        links[slot] = *p;
        *p = slot;
    }

    bool expired () const {
        return tHead != 0 &&
                (uint16_t) (timers[tHead].eVal - ticks - 1) > 60000;
    }

    volatile uint32_t ticks =0;
    Event timers [MAX];
    uint8_t links [MAX], tHead =0, tFree =0, tLast =0, tRate =0;

public:
    enum TAG { TICK, RATE, DELAY };

    uint8_t init () {
        Worker::init();
        setRate(100);
        return wId;
    }

    void irqSysTick () {
        ticks += tRate;
        if (expired())
            trigger(TICK);
    }

    uint32_t millis () const {
        // the result has millisecond resolution, even when rate > 1 ms
        while (true) // spinloop, in case ticks changes midway
            if (uint32_t t = ticks, c = STK[0x8]; t == ticks) {
                return t + tRate - (c*8)/(SystemCoreClock/1000);
            }
    }

    void delay (uint16_t ms, Event done) const {
        assert(done.eDst != 0);
        send({ wId, DELAY, ms }, done);
    }
};

Ticker<10> ticker;

IRQ_HANDLER(SysTick, ticker.irqSysTick)

void testTicker () {
    auto tickerId = ticker.init();
    TEST_ASSERT_GREATER_THAN(0, tickerId);

    // 250x 1 ms busy is 250 ms elapsed, wven with a ticker rate of 100 ms
    for (auto i = 1; i <= 250; ++i) {
        cycles::msBusy(1);
        TEST_ASSERT_INT_WITHIN(1, i, ticker.millis());
    }

    Worker::send({ tickerId, ticker.RATE, 1 });

    // the simpler case is 50x 1 ms when the ticker rate is also 1 ms
    auto start = ticker.millis();
    for (auto i = 1; i <= 50; ++i) {
        cycles::msBusy(1);
        TEST_ASSERT_INT_WITHIN(1, i, ticker.millis()-start);
    }

    // check that 25 interrupts also happen in 25 ms
    start = ticker.millis();
    for (auto i = 1; i <= 25; ++i)
        asm ("wfi");
    TEST_ASSERT_INT_WITHIN(1, 25, ticker.millis()-start);
}

struct SequentialDelays : Worker {
    enum TAG { START, ONE, TWO, THREE };

    uint16_t start;
    bool done =false;

    Event process (Event in, Event out, void*) override {
        switch (in.eTag) {
            case START:
                start = ticker.millis();
                ticker.delay(5, { wId, ONE });
                break;
            case ONE:
                TEST_ASSERT_INT_WITHIN(1, 5, ticker.millis()-start);
                ticker.delay(10, { wId, TWO });
                break;
            case TWO:
                TEST_ASSERT_INT_WITHIN(1, 5+10, ticker.millis()-start);
                ticker.delay(20, { wId, THREE });
                break;
            case THREE:
                TEST_ASSERT_INT_WITHIN(1, 5+10+20, ticker.millis()-start);
                done = true;
                break;
            default:
                fail();
        }
        return out;
    }
};

void testSequentialDelays () {
    SequentialDelays worker;
    auto swId = worker.init();
    auto tickerId = ticker.init();

    TEST_ASSERT_GREATER_THAN(0, swId);
    TEST_ASSERT_GREATER_THAN(swId, tickerId);

    Worker::send({ tickerId, ticker.RATE, 1 });

    // start 3 delays in sequence, for 5, 10, and 20 ms, respectively
    Worker::send({ swId, worker.START });

    int n = 0;
    do {
        asm ("wfi");
        ++n;
    } while (!worker.done);

    // since the ticker runs every 1 ms, there will have been 35 interrupts
    TEST_ASSERT_EQUAL(35, n);
}

struct ParallelDelays : Worker {
    enum TAG { START, ONE, TWO, THREE };

    uint16_t start;
    uint8_t calls =0;

    Event process (Event in, Event out, void*) override {
        ++calls;
        switch (in.eTag) {
            case START:
                start = ticker.millis();
                ticker.delay( 5, { wId, ONE });   // first one
                ticker.delay(30, { wId, TWO });   // appended to end
                ticker.delay(15, { wId, THREE }); // inserted before last
                break;
            case ONE:
                TEST_ASSERT_INT_WITHIN(1, 5, ticker.millis()-start);
                break;
            case TWO:
                TEST_ASSERT_INT_WITHIN(1, 30, ticker.millis()-start);
                break;
            case THREE:
                TEST_ASSERT_INT_WITHIN(1, 15, ticker.millis()-start);
                break;
            default:
                fail();
        }
        return out;
    }
};

void testParallelDelays () {
    ParallelDelays worker;
    auto pwId = worker.init();
    auto tickerId = ticker.init();

    TEST_ASSERT_GREATER_THAN(0, pwId);
    TEST_ASSERT_GREATER_THAN(pwId, tickerId);

    Worker::send({ tickerId, ticker.RATE, 1 });

    // start 3 delays in parallel, for 5, 15, and 30 ms, respectively
    Worker::send({ pwId, worker.START });
    TEST_ASSERT_EQUAL(1, worker.calls);

    int n = 0;
    do {
        asm ("wfi");
        ++n;
    } while (worker.calls < 4);

    // since the ticker runs every 1 ms, there will have been 30 interrupts
    TEST_ASSERT_EQUAL(30, n);
}

void allTests () {
    RUN_TEST(testTicker);
    RUN_TEST(testSequentialDelays);
    RUN_TEST(testParallelDelays);
}
