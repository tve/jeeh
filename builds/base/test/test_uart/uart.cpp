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

uart::Work<UART_TYPE> uartWork (UART_CONF);
IRQ_HANDLER(UART_NAME, uartWork.interrupt)
IRQ_HANDLER(DMA1_Channel1, uartWork.interrupt) // not DMA1_CH1 !
IRQ_HANDLER(DMA1_Channel2, uartWork.interrupt) // not DMA1_CH2 !

void setUp () {}

void tearDown () {
    uartPoll.deinit();
    uartSync.deinit();
    uartWork.deinit();
}

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
    TEST_ASSERT_INT_WITHIN(1, 2, cycles::micros()-start);

    start = cycles::micros();
    uartSync.transfer(true, (uint8_t*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(2, 47, cycles::micros()-start);

    start = cycles::micros();
    uartSync.transfer(true, (uint8_t*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(1, 100, cycles::micros()-start);

    start = cycles::micros();
    uartSync.transfer(true, (uint8_t*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(1, 300, cycles::micros()-start);
}

void testWait () {
    uartWork.init(UART_PINS, 1'000'000);

    auto start = cycles::micros();
    uartWork.transfer(true, (uint8_t*) "x", 1);
    TEST_ASSERT_INT_WITHIN(1, 8, cycles::micros()-start);

    start = cycles::micros();
    uartWork.transfer(true, (uint8_t*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(2, 47, cycles::micros()-start);

    start = cycles::micros();
    uartWork.transfer(true, (uint8_t*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(1, 100, cycles::micros()-start);

    start = cycles::micros();
    uartWork.transfer(true, (uint8_t*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(1, 300, cycles::micros()-start);
}

struct UartWorker : Worker {
    enum TAG { START, ONE, TWO, THREE, FOUR };

    uint8_t calls =0;
    bool done =false;

    using Worker::init;

private:
    Event process (Event in, Event out, void*) override {
        ++calls;

        switch (in.eTag) {
            case START:
                uartWork.start(true, (uint8_t*) "x", 1, { wId, ONE });
                break;
            case ONE:
                uartWork.start(true, (uint8_t*) "abcde", 5, { wId, TWO });
                break;
            case TWO:
                uartWork.start(true, (uint8_t*) "1234567890", 10, { wId, THREE });
                break;
            case THREE:
                uartWork.start(true,
                               (uint8_t*) "123456789012345678901234567890", 30,
                               { wId, FOUR });
                break;
            case FOUR:
                done = true;
                break;
            default:
                fail();
        }
        return out;
    }
};

void testWork () {
    UartWorker worker;
    auto uwId = uartWork.init(UART_PINS, 1'000'000);
    auto wkId = worker.init();

    TEST_ASSERT_GREATER_THAN(0, wkId);
    TEST_ASSERT_GREATER_THAN(wkId, uwId);

    auto start = cycles::micros();
    Worker::send({ wkId, worker.START });
    TEST_ASSERT_GREATER_OR_EQUAL(1, worker.calls); // might already be 2

    int n = 0;
    while (!worker.done) { asm ("wfi"); ++n; }
    TEST_ASSERT_INT_WITHIN(1, 456, cycles::micros()-start); // 1+5+10+30 chars

    TEST_ASSERT_EQUAL(5, worker.calls);
}

void allTests () {
    RUN_TEST(testJumper);
    RUN_TEST(testPoll);
    RUN_TEST(testSync);
    RUN_TEST(testWait); // async in blocking mode (sync-like)
    RUN_TEST(testWork); // async in full non-blocking mode
    RUN_TEST(testSync); // make sure reinit works
    RUN_TEST(testPoll); // make sure reinit works
}
