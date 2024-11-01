// Very basic test of events and workers.

#include "../common.h"

void setUp () {}
void tearDown () {}

struct SimpleWorker : Worker {
    uint8_t calls =0, lastTag =0;
    uint16_t lastVal =0;

    using Worker::init;

private:
    Event process (Event in, Event out) override {
        ++calls;
        lastTag = in.eTag;
        lastVal = in.eVal;

        TEST_ASSERT_EQUAL(wId, level);

        out.eVal *= 2; // change the reply value
        return out;
    }
};

void testSimpleWorker () {
    SimpleWorker w1;

    auto id1 = w1.init();
    TEST_ASSERT_GREATER_THAN(0, id1);

    TEST_ASSERT_EQUAL(0, w1.calls);
    TEST_ASSERT_EQUAL(0, w1.lastTag);
    TEST_ASSERT_EQUAL(0, w1.lastVal);

    Worker::send({ id1, 11 });
    TEST_ASSERT_EQUAL(1, w1.calls);
    TEST_ASSERT_EQUAL(11, w1.lastTag);
    TEST_ASSERT_EQUAL(0, w1.lastVal);

    Worker::send({ id1, 22, 1111 });
    TEST_ASSERT_EQUAL(2, w1.calls);
    TEST_ASSERT_EQUAL(22, w1.lastTag);
    TEST_ASSERT_EQUAL(1111, w1.lastVal); // no change in incoming value

    // request a reply, which also gets sent to w1 in this case
    Worker::send({ id1, 33, 2222 }, { id1, 44, 3333 });
    TEST_ASSERT_EQUAL(4, w1.calls);
    TEST_ASSERT_EQUAL(44, w1.lastTag);
    TEST_ASSERT_EQUAL(6666, w1.lastVal); // reply value was doubled

    SimpleWorker w2;

    auto id2 = w2.init();
    TEST_ASSERT_GREATER_THAN(0, id2);

    TEST_ASSERT_NOT_EQUAL(id1, id2);

    // request a reply, which now gets sent from w1 to w2
    Worker::send({ id1, 55, 321 }, { id2, 66, 123 });
    TEST_ASSERT_EQUAL(5, w1.calls);
    TEST_ASSERT_EQUAL(55, w1.lastTag);
    TEST_ASSERT_EQUAL(321, w1.lastVal);
    TEST_ASSERT_EQUAL(1, w2.calls);
    TEST_ASSERT_EQUAL(66, w2.lastTag);
    TEST_ASSERT_EQUAL(246, w2.lastVal); // reply value was doubled
}

void allTests () {
    RUN_TEST(testSimpleWorker);
}
