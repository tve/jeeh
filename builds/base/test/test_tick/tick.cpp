// System tick tests.

#include "../common.h"

void setUp () {}
void tearDown () { Worker::clearAll(); }

template< uint8_t MAX >
class Ticker : Worker {
    Event process (Event in, Event out, void*) override {
        switch (in.eTag) {
            case TICK:
                while (expired()) {
                    auto h = head;
                    head = links[h];
                    send(timers[h]);
                    timers[h].eVal = free;
                    free = h;
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
        //
        out.eVal = ticks + ms; // deadline
        auto slot = 0;
        if (free != 0) {
            slot = links[free];
            free = timers[slot].eVal;
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

    void interrupt () {
        ticks += rate;
        if (expired())
            //send({ wId, TICK });
            pend({ wId, TICK });
    }

    uint32_t millis () const {
        // the result has millisecond resolution, even when rate > 1 ms
        while (true) // spinloop, in case ticks changes midway
            if (uint32_t t = ticks, c = STK[0x8]; t == ticks) {
                return t + rate - (c*8)/(SystemCoreClock/1000);
            }
    }

    void delay (uint16_t ms, Event reply) const {
        assert(reply.eDst != 0);
        send({ wId, DELAY, ms }, reply);
    }
};

Ticker<10> ticker;

IRQ_HANDLER(SysTick, ticker.interrupt)

void msTicker () {
    auto tickerId = ticker.init();
    TEST_ASSERT_GREATER_THAN(0, tickerId);

    for (auto i = 1; i <= 250; ++i) {
        cycles::msBusy(1);
        TEST_ASSERT_INT_WITHIN(1, i, ticker.millis());
    }

    Worker::send({ tickerId, ticker.RATE, 1 });

    auto t = ticker.millis();
    for (auto i = 1; i <= 50; ++i) {
        cycles::msBusy(1);
        TEST_ASSERT_INT_WITHIN(1, t+i, ticker.millis());
    }
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
done = true;
                //ticker.delay(10, { wId, TWO });
                break;
            case TWO:
                TEST_ASSERT_INT_WITHIN(1, start+5+10, ticker.millis());
                ticker.delay(20, { wId, THREE });
                break;
            case THREE:
                TEST_ASSERT_INT_WITHIN(1, start+5+10+20, ticker.millis());
                done = true;
                break;
            default:
                fail();
        }
        return out;
    }
};

void msWaiter () {
    // TODO worker priority ordering
    auto tickerId = ticker.init();
    TEST_ASSERT_GREATER_THAN(0, tickerId);

    Worker::send({ tickerId, ticker.RATE, 1 });

    TimerWorker tw;
    auto twId = tw.init();
    TEST_ASSERT_GREATER_THAN(0, twId);

    Worker::send({ twId, tw.START });

    int n = 0;
    do {
        asm ("wfi");
        ++n;
    } while (!tw.done);
    TEST_ASSERT_EQUAL(35, n);
}

void allTests () {
    RUN_TEST(msTicker);
    RUN_TEST(msWaiter);
}
