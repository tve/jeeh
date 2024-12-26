// Analog (DAC & ADC) tests.

#include "../common.h"
#include "defs.h"

void setUp () {}
void tearDown () { dac::deinit(); }

void testJumper () {
    Pin outPin (PINS_DAC,"P"), inPin (PINS_ADC,"F");

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
    Pin dacPin (PINS_DAC,"A"), adcPin (PINS_ADC,"A");
    adc::init();
    dac::init();

    // this needs a jumper from PINS_DAC to PINS_ADC2
    // the bottom and top 1% of the DAC range are not very accurate
    for (auto i = 40; i < 4096-40; ++i) {
        dac::set(i);
        cycles::usBusy(2);
        auto v = adc::read(PINS_ACH);
        //logf("%d = %d - %d", i-v, i, v);
        // there are some variations, but ± 60+1.5% should be ok
        // most irregularities appear to be in the middle of the range
        TEST_ASSERT_INT_WITHIN(60+i/64, i, v);
    }
}

void allTests () {
    RUN_TEST(testJumper);
    RUN_TEST(testDac);
    RUN_TEST(testAdc);
}
