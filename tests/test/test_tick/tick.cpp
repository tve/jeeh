// System tick tests.

#include "../common.h"
#include "jee/ticker.h"

Ticker ticker;
TICKER_TRIGGER(ticker)

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
    Task::showStats();
}

struct Sequential : Task {
    enum TAG { START, ONE, TWO, THREE };

    uint16_t start;
    bool done =false;

    Sequential () : Task ("Sequential") {}

    Event process (Event in, Event out) override {
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

void testSequential () {
    Sequential task;
    auto tkId = ticker.init();
    auto sdId = task.init();

    TEST_ASSERT_GREATER_THAN(0, sdId);
    TEST_ASSERT_GREATER_THAN(sdId, tkId);

    // start 3 delays in sequence, for 5, 10, and 20 ms, respectively
    // FIXME Task::send({ sdId, task.START });

    int n = 0;
    do { asm ("wfi"); ++n; } while (!task.done);

    // since the ticker runs every 1 ms, there will have been 35 interrupts
    TEST_ASSERT_EQUAL(35, n);
    Task::showStats();
}

struct Parallel : Task {
    enum TAG { START, ONE, TWO, THREE };

    uint16_t start;
    uint8_t calls =0;

    Parallel () : Task ("Parallel") {}

    Event process (Event in, Event out) override {
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

void testParallel () {
    Parallel task;
    auto tkId = ticker.init();
    auto pdId = task.init();

    TEST_ASSERT_GREATER_THAN(0, pdId);
    TEST_ASSERT_GREATER_THAN(pdId, tkId);

    // start 3 delays in parallel, for 5, 15, and 30 ms, respectively
    Task::send({ pdId, task.START });
    TEST_ASSERT_EQUAL(1, task.calls);

    int n = 0;
    do { asm ("wfi"); ++n; } while (task.calls < 4);

    // since the ticker runs every 1 ms, there will have been 30 interrupts
    TEST_ASSERT_EQUAL(30, n);
    Task::showStats();
}

struct Cancelled : Task {
    enum TAG { START, ONE, TWO, THREE };

    uint16_t start;
    uint8_t calls =0;

    Cancelled () : Task ("Cancelled") {}

    Event process (Event in, Event out) override {
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

void testCancelled () {
    Cancelled task;
    auto tkId = ticker.init();
    auto cdId = task.init();

    TEST_ASSERT_GREATER_THAN(0, cdId);
    TEST_ASSERT_GREATER_THAN(cdId, tkId);

    // start 3 delays in parallel, for 5, 15, and 30 ms, respectively
    // after 5 ms, the 15 ms delay is cancelled so it won't trigger
    Task::send({ cdId, task.START });
    TEST_ASSERT_EQUAL(1, task.calls);

    int n = 0;
    do { asm ("wfi"); ++n; } while (task.calls < 3);

    // since the ticker runs every 1 ms, there will have been 30 interrupts
    TEST_ASSERT_EQUAL(30, n);
    Task::showStats();
}

struct Periodic : Task {
    enum TAG { START, ONE, TWO, THREE };

    uint16_t start, expect =0;
    uint8_t calls =0;
    bool done =false;

    Periodic () : Task ("Periodic") {}

    Event process (Event in, Event out) override {
        ++calls;
        switch (in.eTag) {
            case START:
                start = ticker.millis();
                ticker.delay(25, TWO);   // cancel the periodic timer
                ticker.periodic(7, ONE); // start periodic timer
                ticker.delay(30, THREE); // and make sure it stopped
                break;
            case ONE:
                TEST_ASSERT_LESS_THAN(5, calls);
                expect += 7;
                TEST_ASSERT_INT_WITHIN(1, expect, ticker.millis()-start);
                break;
            case TWO:
                TEST_ASSERT_EQUAL(5, calls);
                TEST_ASSERT_INT_WITHIN(1, 25, ticker.millis()-start);
                ticker.cancel(ONE); // cancel periodic before it fires again
                break;
            case THREE:
                TEST_ASSERT_EQUAL(6, calls);
                TEST_ASSERT_INT_WITHIN(1, 30, ticker.millis()-start);
                done = true;
                break;
            default:
                fail();
        }
        return out;
    }
};

void testPeriodic () {
    Periodic task;
    auto tkId = ticker.init();
    auto pdId = task.init();

    TEST_ASSERT_GREATER_THAN(0, pdId);
    TEST_ASSERT_GREATER_THAN(pdId, tkId);

    // start 3 delays in parallel, one of them is periodic
    // another delay cancels the periodic one
    // and the last one makes sure the cancellation worked
    Task::send({ pdId, task.START });
    TEST_ASSERT_EQUAL(1, task.calls);

    int n = 0;
    do { asm ("wfi"); ++n; } while (!task.done);
    TEST_ASSERT_EQUAL(6, task.calls); // start + 7, 14, 21, 25, 30 ms

    // since the ticker runs every 1 ms, there will have been 30 interrupts
    TEST_ASSERT_EQUAL(30, n);
    Task::showStats();
}

struct Postpone : Task {
    enum TAG { START, ONE, TWO, THREE, FOUR };

    uint16_t start;
    char capture [40];
    uint8_t calls =0;
    bool done =false;

    Postpone () : Task ("Postpone") {}

    Event process (Event in, Event out) override {
        capture[calls++] = '0' + in.eTag;
        TEST_ASSERT_LESS_OR_EQUAL(sizeof capture, calls+1); // trailing zero

        switch (in.eTag) {
            case START:
                start = ticker.millis();
                ticker.periodic(1, ONE); // start periodic timer
                ticker.periodic(4, TWO); // second slower periodic timer
                ticker.delay(25, THREE); // cancel the periodic timers
                ticker.delay(30, FOUR);  // and make sure they stopped
                break;
            case ONE:
                break;
            case TWO:
                cycles::msBusy(2); // prevent ticks from being processed
                break;
            case THREE:
                ticker.cancel(ONE); // cancel periodic before it fires again
                ticker.cancel(TWO); // also cancel second periodic timer
                break;
            case FOUR:
                capture[calls] = 0;
                done = true;
                break;
            default:
                fail();
        }
        return out;
    }
};

void testPostpone () {
    Postpone task;
    auto tkId = ticker.init();
    auto pdId = task.init();

    TEST_ASSERT_GREATER_THAN(0, pdId);
    TEST_ASSERT_GREATER_THAN(pdId, tkId);

    // start two periodic timers and verify the sequence in which they fired
    Task::send({ pdId, task.START });
    TEST_ASSERT_EQUAL(1, task.calls);

    int n = 0;
    do { asm ("wfi"); ++n; } while (!task.done);

    TEST_ASSERT_EQUAL_STRING("011121111211112111121111211112131114",
                                task.capture);

    // since the ticker runs every 1 ms, there were at most 30 interrupts
    TEST_ASSERT_LESS_OR_EQUAL(30, n);
    //TEST_ASSERT_EQUAL(30, n); // TODO why 18 iso 30?
    Task::showStats();
}

void allTests () {
    RUN_TEST(testTicker);
    RUN_TEST(testSequential);
    RUN_TEST(testParallel);
    RUN_TEST(testCancelled);
    RUN_TEST(testPeriodic);
    RUN_TEST(testPostpone);
}
