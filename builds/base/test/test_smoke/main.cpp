// A minimal test, no smoke will appear if all is well ...

#include "../common.h"

void setUp () {}
void tearDown () {}

void smoke () {
    TEST_ASSERT_EQUAL(42, 40 + 2); // verify that this trivial test is working
}

void allTests () {
    UNITY_BEGIN();
    RUN_TEST(smoke);
}
