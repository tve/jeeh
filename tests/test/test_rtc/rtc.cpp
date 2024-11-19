// Test the built-in Real Time Clock, running on LSI @ 40 kHz.

#include "../common.h"

void setUp () {
    rtc::init(false); // no LSE present on G431KB, must use LSI clock
}

void tearDown () {
    rtc::deinit();
    rtc::reset(); // completely reset the RTC + backup domain
}

void rtc_isRunning () {
    auto dt = rtc::getDate();
    TEST_ASSERT_EQUAL(dt.ms, rtc::getDate().ms);

    cycles::msBusy(5); // wait slightly longer than one 256 Hz clock tick
    TEST_ASSERT_NOT_EQUAL(dt.ms, rtc::getDate().ms);
}

void rtc_todMillis () {
    auto ms = rtc::getDate().todMillis();

    cycles::msBusy(500);
    auto t = rtc::getDate().todMillis() - ms;

    TEST_ASSERT_INT_WITHIN(20, 500, t); // RTC time should match cycles time
}

void rtc_oneSecond () {
    auto dt = rtc::getDate();
    while (rtc::getDate().ms == dt.ms) {} // wait until RTC clock advances

    cycles::clear();
    while (rtc::getDate().ms != dt.ms) {} // wait until it has stepped 256x
    auto ms = cycles::millis();

    TEST_ASSERT_INT_WITHIN(40, 1000, ms); // LSI clock is not very accurate
    TEST_ASSERT_EQUAL_UINT32(dt + 1, rtc::getSecs());
}

void allTests () {
    RUN_TEST(rtc_isRunning);
    RUN_TEST(rtc_todMillis);    // this takes ≈500 ms
    RUN_TEST(rtc_oneSecond);    // this takes ≈1000 ms
}
