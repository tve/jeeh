#include <unity.h>
#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

void setUp () {
    rtc::init(false); // no LSE pins, must use LSI clock
}

void tearDown () {
    rtc::deinit();
    rtc::reset(); // completely reset the RTC + backup domain
}

void rtc_isRunning () {
    auto dt = rtc::getDate();
    TEST_ASSERT_EQUAL(dt.ff, rtc::getDate().ff);

    cycles::msBusy(5);
    TEST_ASSERT_NOT_EQUAL(dt.ff, rtc::getDate().ff);
}

void rtc_todMillis () {
    auto ms = rtc::getDate().todMillis();

    cycles::msBusy(500);
    auto t = rtc::getDate().todMillis() - ms;

    TEST_ASSERT_INT_WITHIN(20, 500, t);
}

void rtc_oneSecond () {
    auto dt = rtc::getDate();
    while (rtc::getDate().ff == dt.ff) {}

    cycles::clear();
    while (rtc::getDate().ff != dt.ff) {}
    auto ms = cycles::millis();

    TEST_ASSERT_INT_WITHIN(40, 1000, ms); // LSI clock is not very accurate
    TEST_ASSERT_EQUAL_UINT32(dt + 1, rtc::getSecs());
}

int main () {
    fastClock();
    cycles::init();

    UNITY_BEGIN();
    RUN_TEST(rtc_isRunning);
    RUN_TEST(rtc_todMillis);    // this takes ≈500 ms
    RUN_TEST(rtc_oneSecond);    // this takes ≈1000 ms
    return UNITY_END();
}
