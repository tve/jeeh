// DMA-based UART tests.

#include "common.h"
#include "defs.h"
#include "jee/ticker.h"

Ticker ticker;
TICKER_INSTALL(ticker)

uart::Poll<UART2_NAME.ADDR> uartPoll (ena::UART2_NAME, UART2_FREQ);

void setUp () {}
void tearDown () { uartPoll.deinit(); }

void testJumper () {
    Pin outPin ("A9","P"), inPin ("A10","F");

    // check that the two pins are connected via a jumper
    TEST_ASSERT_EQUAL(0, inPin);
    outPin = 1;
    TEST_ASSERT_EQUAL(1, inPin);
}

void testPoll () {
    uartPoll.init(UART2_PINS, 1'000'000);

    cycles::clear();
    uartPoll.transfer(true, (void*) "x", 1);
    TEST_ASSERT_INT_WITHIN(1, 20, cycles::micros());

    cycles::clear();
    uartPoll.transfer(true, (void*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(2, 50, cycles::micros());

    cycles::clear();
    uartPoll.transfer(true, (void*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(5, 100, cycles::micros());

    cycles::clear();
    uartPoll.transfer(true, (void*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(10, 300, cycles::micros());
}

void allTests () {
    RUN_TEST(testJumper);
    RUN_TEST(testPoll);
}
