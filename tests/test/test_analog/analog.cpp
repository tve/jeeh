// Analog (DAC & ADC) tests.

#include "../common.h"
#include "jee/adc.h"
#include "jee/dac.h"

void setUp () {}
void tearDown () { dac::deinit(); }

void testJumper () {
    Pin outPin ("A4","P"), inPin ("A7","F");

    // check that the two pins are connected via a jumper
    TEST_ASSERT_EQUAL(0, inPin);
    outPin = 1;
    TEST_ASSERT_EQUAL(1, inPin);

    outPin.mode("F");
}

void testDac () {
    dac::init();

    for (auto i = 0; i < 4096; i += 16) {
        dac::set(i);
        cycles::usBusy(2);
    }
}

void testAdc () {
    Pin::config("A4:A,A7");
    adc::init();
    dac::init();

    // this needs a jumper from PA4 (DAC) to PA7 (ADC2 ch 4)
    // the bottom and top 1% of the DAC range are not very accurate
    for (auto i = 40; i < 4096-40; ++i) {
        dac::set(i);
        cycles::usBusy(2);
        auto v = adc::read(4); // PA7
        //logf("%d = %d - %d", i-v, i, v);
        // there are some variations, but ± 48+1.5% should be ok
        // most irregularities appear to be in the middle of the range
        TEST_ASSERT_INT_WITHIN(48+i/64, i, v);
    }
}

void allTests () {
    RUN_TEST(testJumper);
    RUN_TEST(testDac);
    RUN_TEST(testAdc);
}
