// DMA-based UART tests.

#include "common.h"
#include "jee/ticker.h"
//#include "defs.h"

#define UART_NAME  USART1
#define UART_PINS  "A9:U7,A10"
#define UART_FREQ  170
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,25,24

Ticker ticker;
TICKER_INSTALL(ticker)

uart::Poll<UART_NAME.ADDR> uartPoll (ena::UART_NAME, UART_FREQ);

#define UART_TYPE  UART_NAME.ADDR,DMA1.ADDR,1-1,2-1
#define UART_OCONF  { ena::UART_NAME,170,Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,25,24 }

uart::Sync<UART_TYPE> uartSync (UART_OCONF);
IRQ_HANDLER(DMA1_CH1, uartSync.interrupt)
IRQ_HANDLER(DMA1_CH2, uartSync.interrupt)
IRQ_HANDLER(UART_NAME, uartSync.interrupt)

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
    uartPoll.transfer(true, (void*) "x", 1);
    TEST_ASSERT_INT_WITHIN(4, 20, cycles::micros()-start);

    start = cycles::micros();
    uartPoll.transfer(true, (void*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(4, 50, cycles::micros()-start);

    start = cycles::micros();
    uartPoll.transfer(true, (void*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(5, 100, cycles::micros()-start);

    start = cycles::micros();
    uartPoll.transfer(true, (void*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(10, 300, cycles::micros()-start);
}

void testSync () {
    uartSync.init(UART_PINS, 1'000'000);

    auto start = cycles::micros();
    uartSync.transfer(true, (void*) "x", 1);
    TEST_ASSERT_INT_WITHIN(1, 20, cycles::micros()-start);

    start = cycles::micros();
    uartSync.transfer(true, (void*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(2, 50, cycles::micros()-start);

    start = cycles::micros();
    uartSync.transfer(true, (void*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(5, 100, cycles::micros()-start);

    start = cycles::micros();
    uartSync.transfer(true, (void*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(10, 300, cycles::micros()-start);
}

void allTests () {
    RUN_TEST(testJumper);
    RUN_TEST(testPoll);
    RUN_TEST(testSync);
}
