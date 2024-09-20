// Very basic test of events and workers.

#include "../common.h"

void setUp () {}
void tearDown () {}

class SimpleWorker : Worker {
    Event process (Event in, Event out, void*) override {
        ++calls;
        lastTag = in.eTag;
        lastVal = in.eVal;

        TEST_ASSERT_EQUAL(this, current);

        out.eVal *= 2; // change the reply value
        return out;
    }

public:
    using Worker::init;

    uint8_t calls =0, lastTag =0;
    uint16_t lastVal =0;
};

void simpleWorker () {
    SimpleWorker w;

    auto wid = w.init();
    TEST_ASSERT_GREATER_THAN(0, wid);
    TEST_ASSERT_EQUAL(&w, &Worker::at(wid));

    TEST_ASSERT_EQUAL(0, w.calls);
    TEST_ASSERT_EQUAL(0, w.lastTag);
    TEST_ASSERT_EQUAL(0, w.lastVal);

    Worker::send ({ wid, 11 });
    TEST_ASSERT_EQUAL(1, w.calls);
    TEST_ASSERT_EQUAL(11, w.lastTag);
    TEST_ASSERT_EQUAL(0, w.lastVal);

    Worker::send ({ wid, 22, 1111 });
    TEST_ASSERT_EQUAL(2, w.calls);
    TEST_ASSERT_EQUAL(22, w.lastTag);
    TEST_ASSERT_EQUAL(1111, w.lastVal); // no change in incoming value

    // request a reply, which also gets sent to w in this case
    Worker::send ({ wid, 33, 2222 }, { wid, 44, 3333 });
    TEST_ASSERT_EQUAL(4, w.calls);
    TEST_ASSERT_EQUAL(44, w.lastTag);
    TEST_ASSERT_EQUAL(6666, w.lastVal); // reply value was doubled

    SimpleWorker w2;

    auto wid2 = w2.init();
    TEST_ASSERT_GREATER_THAN(0, wid2);
    TEST_ASSERT_EQUAL(&w2, &Worker::at(wid2));

    TEST_ASSERT_NOT_EQUAL(wid, wid2);

    // request a reply, which now gets sent from w to w2
    Worker::send ({ wid, 55, 321 }, { wid2, 66, 123 });
    TEST_ASSERT_EQUAL(5, w.calls);
    TEST_ASSERT_EQUAL(55, w.lastTag);
    TEST_ASSERT_EQUAL(321, w.lastVal);
    TEST_ASSERT_EQUAL(1, w2.calls);
    TEST_ASSERT_EQUAL(66, w2.lastTag);
    TEST_ASSERT_EQUAL(246, w2.lastVal); // reply value was doubled
}

void allTests () {
    RUN_TEST(simpleWorker);
}
