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
                    auto h = head;
                    head = links[h];
                    send(timers[h]);
                    // FIXME
                    //links[h] = free;
                    //free = h;
                }
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
        rate = ms;
        STK[0x4] = (rate * (SystemCoreClock/1000)) / 8 - 1; // reload value
        STK[0x8] = 0;
        STK[0x0] = 0b011; // enable, clk/8 mode
    }

    void add (uint16_t ms, Event out) {
        out.eVal = ticks + ms; // deadline
        auto slot = 0;
        if (free != 0) {
            slot = links[free];
            free = links[slot];
        } else
            slot = ++last;
        links[slot] = head;
        head = slot;
        timers[head] = out;
    }

    bool expired () const {
        return head != 0 && (uint16_t) (timers[head].eVal - ticks - 1) > 60000;
    }

    uint8_t rate =0;
    volatile uint32_t ticks =0;
    Event timers [MAX];
    uint8_t links [MAX], head =0, free =0, last =0;

public:
    enum TAG { TICK, RATE, DELAY };

    uint8_t init () {
        Worker::init();
        setRate(100);
        return wId;
    }

    void irqSysTick () {
        ticks += rate;
        if (expired())
            trigger(TICK);
    }

    uint32_t millis () const {
        // the result has millisecond resolution, even when rate > 1 ms
        while (true) // spinloop, in case ticks changes midway
            if (uint32_t t = ticks, c = STK[0x8]; t == ticks) {
                return t + rate - (c*8)/(SystemCoreClock/1000);
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

struct TimerWorker : Worker {
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
                TEST_ASSERT_INT_WITHIN(1, start+5, ticker.millis());
                ticker.delay(10, { wId, TWO });
                break;
            case TWO:
                TEST_ASSERT_INT_WITHIN(1, start+5+10, ticker.millis());
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

TimerWorker tw;

void testTimerWorker () {
    auto twId = tw.init();
    auto tickerId = ticker.init();

    TEST_ASSERT_GREATER_THAN(0, twId);
    TEST_ASSERT_GREATER_THAN(twId, tickerId);

    Worker::send({ tickerId, ticker.RATE, 1 });

    // start 3 delays in sequence, for 5, 10, and 20 ms, respectively
    Worker::send({ twId, tw.START });

    int n = 0;
    do {
        asm ("wfi");
        ++n;
    } while (!tw.done);

    // since the ticker runs every 1 ms, there will have been 35 interrupts
    TEST_ASSERT_EQUAL(35, n);
}

void allTests () {
    RUN_TEST(testTicker);
    RUN_TEST(testTimerWorker);
}
