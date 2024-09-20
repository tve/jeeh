#include <unity.h>
#include <jee.h>
using namespace jeeh;

void setUp () {}
void tearDown () {}

void smoke () {
    TEST_ASSERT_EQUAL(42, 40 + 2);
}

int main () {
    fastClock();
    UNITY_BEGIN();
    RUN_TEST(smoke);
    return UNITY_END();
}
