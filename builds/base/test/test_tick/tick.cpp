// System tick tests.

#include "../common.h"

void setUp () {}
void tearDown () {}

class TickWorker : Worker {
    Event process (Event in, Event out, void*) override {
        switch (in.eTag) {
            case TICK:
                break;
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

    uint8_t rate =0;
    volatile uint32_t ticks =0;

public:
    enum TAG { TICK };

    uint8_t init () {
        Worker::init();
        setRate(100);
        return wId;
    }

    void interrupt () {
        ticks += rate;
        send({ wId, TICK, rate });
    }

    uint32_t millis () const {
        // the result has millisecond resolution, even when rate > 1 ms
        while (true) // spinloop, in case ticks changes midway
            if (uint32_t t = ticks, c = STK[0x8]; t == ticks) {
                return t + rate - (c*8)/(SystemCoreClock/1000);
            }
    }
};

TickWorker tw;

IRQ_HANDLER(SysTick, tw.interrupt)

void tickWorker () {
    auto wid = tw.init();
    TEST_ASSERT_GREATER_THAN(0, wid);

    for (auto i = 1; i <= 250; ++i) {
        cycles::msBusy(1);
        TEST_ASSERT_INT_WITHIN(1, i, tw.millis());
    }
}

void allTests () {
    RUN_TEST(tickWorker);
}
