// System tick tests.

#include "../common.h"

void setUp () {}
void tearDown () {}

void testTicker () {
    auto tkId = ticker.init();
    TEST_ASSERT_GREATER_THAN(0, tkId);

    // 250x 1 ms busy is 250 ms elapsed, even with a ticker rate of 100 ms
    for (auto i = 1; i <= 250; ++i) {
        cycles::msBusy(1);
        TEST_ASSERT_INT_WITHIN(1, i, ticker.millis());
    }

    Worker::send({ tkId, ticker.RATE, 1 });

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

struct SequentialDelay : Worker {
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

void testSequentialDelay () {
    SequentialDelay worker;
    auto tkId = ticker.init();
    auto swId = worker.init();

    TEST_ASSERT_GREATER_THAN(0, swId);
    TEST_ASSERT_GREATER_THAN(swId, tkId);

    Worker::send({ tkId, ticker.RATE, 1 });

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

struct ParallelDelay : Worker {
    enum TAG { START, ONE, TWO, THREE };

    uint16_t start;
    uint8_t calls =0;

    Event process (Event in, Event out, void*) override {
        ++calls;
        switch (in.eTag) {
            case START:
                start = ticker.millis();
                ticker.delay( 5, ONE);   // first one
                ticker.delay(30, TWO);   // appended to end
                ticker.delay(15, THREE); // inserted before last
                break;
            case ONE:
                TEST_ASSERT_EQUAL(2, calls);
                TEST_ASSERT_INT_WITHIN(1, 5, ticker.millis()-start);
                break;
            case TWO:
                TEST_ASSERT_EQUAL(4, calls);
                TEST_ASSERT_INT_WITHIN(1, 30, ticker.millis()-start);
                break;
            case THREE:
                TEST_ASSERT_EQUAL(3, calls);
                TEST_ASSERT_INT_WITHIN(1, 15, ticker.millis()-start);
                break;
            default:
                fail();
        }
        return out;
    }
};

void testParallelDelay () {
    ParallelDelay worker;
    auto tkId = ticker.init();
    auto pwId = worker.init();

    TEST_ASSERT_GREATER_THAN(0, pwId);
    TEST_ASSERT_GREATER_THAN(pwId, tkId);

    Worker::send({ tkId, ticker.RATE, 1 });

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

struct CancelledDelay : Worker {
    enum TAG { START, ONE, TWO, THREE };

    uint16_t start;
    uint8_t calls =0;

    Event process (Event in, Event out, void*) override {
        ++calls;
        switch (in.eTag) {
            case START:
                start = ticker.millis();
                ticker.delay( 5, ONE);   // first one
                ticker.delay(30, TWO);   // appended to end
                ticker.delay(15, THREE); // inserted before last
                break;
            case ONE:
                TEST_ASSERT_EQUAL(2, calls);
                TEST_ASSERT_INT_WITHIN(1, 5, ticker.millis()-start);
                ticker.cancel(THREE); // cancel delay before it fires
                break;
            case TWO:
                TEST_ASSERT_EQUAL(3, calls);
                TEST_ASSERT_INT_WITHIN(1, 30, ticker.millis()-start);
                break;
            case THREE:
                TEST_FAIL(); // oops, the cancellation failed
                break;
            default:
                fail();
        }
        return out;
    }
};

void testCancelledDelay () {
    CancelledDelay worker;
    auto tkId = ticker.init();
    auto cwId = worker.init();

    TEST_ASSERT_GREATER_THAN(0, cwId);
    TEST_ASSERT_GREATER_THAN(cwId, tkId);

    Worker::send({ tkId, ticker.RATE, 1 });

    // start 3 delays in parallel, for 5, 15, and 30 ms, respectively
    // after 5 ms, the 15 ms delay is cancelled so it won't trigger
    Worker::send({ cwId, worker.START });
    TEST_ASSERT_EQUAL(1, worker.calls);

    int n = 0;
    do {
        asm ("wfi");
        ++n;
    } while (worker.calls < 3);

    // since the ticker runs every 1 ms, there will have been 30 interrupts
    TEST_ASSERT_EQUAL(30, n);
}

void allTests () {
    RUN_TEST(testTicker);
    RUN_TEST(testSequentialDelay);
    RUN_TEST(testParallelDelay);
    RUN_TEST(testCancelledDelay);
}
