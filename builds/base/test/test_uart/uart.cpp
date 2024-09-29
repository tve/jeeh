// DMA-based UART tests.

#include "common.h"
#include "jee/ticker.h"
#include "jee/uart.h"
#include "defs.h"

Ticker ticker;
TICKER_INSTALL(ticker)

uart::Poll<UART_NAME.ADDR> uartPoll (ena::UART_NAME, UART_FREQ);

// TODO these updated definitions are needed for jee/uart.h
#define UART_TYPE  UART_NAME.ADDR,DMA1.ADDR,1-1,2-1
#undef UART_CONF
#define UART_CONF  { ena::UART_NAME, 170, Irq::UART_NAME, \
                     Irq::DMA1_CH1, Irq::DMA1_CH2, 1-1, 25, 24 }

uart::Sync<UART_TYPE> uartSync (UART_CONF);
IRQ_HANDLER(UART_NAME, uartSync.interrupt)
IRQ_HANDLER(DMA1_Channel1, uartSync.interrupt) // not DMA1_CH1 !
IRQ_HANDLER(DMA1_Channel2, uartSync.interrupt) // not DMA1_CH2 !

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
    uartPoll.init(UART_PINS, 1'000'000);

    auto start = cycles::micros();
    uartPoll.transfer(true, (uint8_t*) "x", 1);
    TEST_ASSERT_INT_WITHIN(1, 20, cycles::micros()-start);

    start = cycles::micros();
    uartPoll.transfer(true, (uint8_t*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(1, 52, cycles::micros()-start);

    start = cycles::micros();
    uartPoll.transfer(true, (uint8_t*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(1, 102, cycles::micros()-start);

    start = cycles::micros();
    uartPoll.transfer(true, (uint8_t*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(1, 307, cycles::micros()-start);
}

void testSync () {
    uartSync.init(UART_PINS, 1'000'000);

    auto start = cycles::micros();
    uartSync.transfer(true, (uint8_t*) "x", 1);
    TEST_ASSERT_INT_WITHIN(1, 6, cycles::micros()-start);

    start = cycles::micros();
    uartSync.transfer(true, (uint8_t*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(1, 46, cycles::micros()-start);

    start = cycles::micros();
    uartSync.transfer(true, (uint8_t*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(1, 100, cycles::micros()-start);

    start = cycles::micros();
    uartSync.transfer(true, (uint8_t*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(1, 300, cycles::micros()-start);
}

void allTests () {
    RUN_TEST(testJumper);
    RUN_TEST(testPoll);
    RUN_TEST(testSync);
}
