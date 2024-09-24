// External pin interrupt tests.

#include "../common.h"

Ticker ticker;
TICKER_INSTALL(ticker)

ExtIrq exti;
EXTIRQ_INSTALL(exti)

void setUp () {}
void tearDown () {}

void testJumper () {
    Pin pins [2];
    Pin::config("A9,A10", pins, sizeof pins);
    pins[0].mode("P");
    pins[1].mode("F");

    // check that the two pins are connected via a jumper
    TEST_ASSERT_EQUAL(0, pins[1]);
    pins[0] = 1;
    TEST_ASSERT_EQUAL(1, pins[1]);
}

struct ExtIinterrupt : Worker {
    enum TAG { START, ONE, TWO, THREE };

    uint16_t start;
    bool done =false;

    Event process (Event in, Event out, void*) override {
        switch (in.eTag) {
            case START:
                start = ticker.millis();
                ticker.delay(5, ONE);
                break;
            case ONE:
                TEST_ASSERT_INT_WITHIN(1, 5, ticker.millis()-start);
                ticker.delay(10, TWO);
                break;
            case TWO:
                TEST_ASSERT_INT_WITHIN(1, 5+10, ticker.millis()-start);
                ticker.delay(20, THREE);
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

void testExti () {
    ExtIinterrupt worker;
    auto tkId = ticker.init();
    auto exId = worker.init();

    TEST_ASSERT_GREATER_THAN(0, exId);
    TEST_ASSERT_GREATER_THAN(exId, tkId);

    Worker::send({ tkId, ticker.RATE, 1 });

// TODO exti test code, replace what follows ===================================

    // start 3 delays in sequence, for 5, 10, and 20 ms, respectively
    Worker::send({ exId, worker.START });

    int n = 0;
    do {
        asm ("wfi");
        ++n;
    } while (!worker.done);

    // since the ticker runs every 1 ms, there will have been 35 interrupts
    TEST_ASSERT_EQUAL(35, n);
}

void allTests () {
    RUN_TEST(testJumper);
    RUN_TEST(testExti);
}
