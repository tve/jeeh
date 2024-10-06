// Random Number Generator tests.

#include "../common.h"
#include "jee/rng.h"

void setUp () {}
void tearDown () {}

void testRandom () {
    rng::init();

    auto start = cycles::micros();
    auto v = rng::rand();
    for (auto i = 0; i < 1000; ++i) {
        auto r = rng::rand();
        TEST_ASSERT_NOT_EQUAL(v, r);
        v = r;
    }
    TEST_ASSERT_INT_WITHIN(1, 358, cycles::micros()-start);

    constexpr auto N = 50;
    rng::Permutation<N> p;

    p.init(); // random permutation of 0..49
    auto sum1 = 0, count = 0;
    while (true) {
        auto r = p.next();
        if (r < 0)
            break;
        sum1 += r;
        ++count;
    }
    TEST_ASSERT_EQUAL(N, count);
    TEST_ASSERT_EQUAL(N*(N-1)/2, sum1); // sum of 0..49, in any order

    p.init();
    p.shuffle();
    auto sum2 = 0;
    for (auto e : p.choice)
        sum2 += e;
    TEST_ASSERT_EQUAL(N*(N-1)/2, sum2); // same result, different algorithm
}

void allTests () {
    RUN_TEST(testRandom);
}
