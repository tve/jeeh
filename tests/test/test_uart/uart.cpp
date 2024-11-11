// DMA-based UART tests.

#include "common.h"
#include "jee/ticker.h"
#include "jee/uart.h"
#include "defs.h"

constexpr auto MARGIN = 10000; // non-zero loosens microsecond timing checks

Ticker ticker;
TICKER_INSTALL(ticker)

uart::Poll<UART_NAME.ADDR> uartPoll (ena::UART_NAME, UART_FREQ);

uart::Sync<UART_TYPE> uartSync (UART_CONF);

uart::Work<UART_TYPE> uartWork (UART_CONF);
UART_INSTALL(uartWork)

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
    TEST_ASSERT_INT_WITHIN(MARGIN, 20, cycles::micros()-start);

    start = cycles::micros();
    uartPoll.transfer(true, (uint8_t*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(MARGIN, 52, cycles::micros()-start);

    start = cycles::micros();
    uartPoll.transfer(true, (uint8_t*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(MARGIN, 102, cycles::micros()-start);

    start = cycles::micros();
    uartPoll.transfer(true, (uint8_t*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(MARGIN, 307, cycles::micros()-start);
}

void testSync () {
    uartSync.init(UART_PINS, 1'000'000);

    auto start = cycles::micros();
    uartSync.transfer(true, (uint8_t*) "x", 1);
    TEST_ASSERT_INT_WITHIN(MARGIN, 2, cycles::micros()-start);

    start = cycles::micros();
    uartSync.transfer(true, (uint8_t*) "abcde", 5);
    TEST_ASSERT_INT_WITHIN(MARGIN, 47, cycles::micros()-start);

    start = cycles::micros();
    uartSync.transfer(true, (uint8_t*) "1234567890", 10);
    TEST_ASSERT_INT_WITHIN(MARGIN, 100, cycles::micros()-start);

    start = cycles::micros();
    uartSync.transfer(true, (uint8_t*) "123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(MARGIN, 300, cycles::micros()-start);
}

void testWait () {
    uartWork.init(UART_PINS, 1'000'000);

    auto start = cycles::micros();
    uartWork.write("x", 1);
    TEST_ASSERT_INT_WITHIN(MARGIN, 3, cycles::micros()-start);

    start = cycles::micros();
    uartWork.write("abcde", 5);
    TEST_ASSERT_INT_WITHIN(MARGIN, 51, cycles::micros()-start);

    start = cycles::micros();
    uartWork.write("1234567890", 10);
    TEST_ASSERT_INT_WITHIN(MARGIN, 99, cycles::micros()-start);

    start = cycles::micros();
    uartWork.write("123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(MARGIN, 300, cycles::micros()-start);
}

struct UartWorker : Worker {
    enum TAG { START, ONE, TWO, THREE, FOUR };

    uint8_t calls =0;
    bool done =false;

    using Worker::init;

private:
    Event process (Event in, Event out) override {
        ++calls;

        switch (in.eTag) {
            case START:
                uartWork.write("x", 1, { wId, ONE });
                break;
            case ONE:
                uartWork.write("abcde", 5, { wId, TWO });
                break;
            case TWO:
                uartWork.write("1234567890", 10, { wId, THREE });
                break;
            case THREE:
                uartWork.write("123456789012345678901234567890", 30,
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
    TEST_ASSERT_INT_WITHIN(MARGIN, 456, cycles::micros()-start); // 1+5+10+30 ch

    TEST_ASSERT_EQUAL(5, worker.calls);
}

struct LoopWorker : Worker {
    enum TAG { START, MORE, SENT, RECV };

    uint8_t calls =0, count =0;
    uint16_t sum =0;
    bool txDone =false, rxDone =false;

    using Worker::init;

private:
    Event process (Event in, Event out) override {
        ++calls;

        switch (in.eTag) {
            case START:
                uartWork.read(0, { wId, RECV });
                [[fallthrough]];
            case MORE:
                ++count;
                // send 1 + 2 + 3 + ... + 25 + 26 + 27 bytes
                uartWork.write("~ABCDEFGHIJKLMNOPQRSTUVWXYZ", count,
                               { wId, count < 27 ? MORE : SENT });
                break;
            case SENT:
                txDone = true;
                break;
            case RECV:
                // count the number of bytes received
                sum += in.eVal;
                if (sum >= 27*28/2) {
                    uartWork.read(in.eVal, {}); // consume without new request
                    rxDone = true;
                } else // keep reading
                    uartWork.read(in.eVal, { wId, RECV });
                break;
            default:
                fail();
        }
        return out;
    }
};

void testLoop () {
    LoopWorker worker;
    auto uwId = uartWork.init(UART_PINS, 1'000'000);
    auto wkId = worker.init();

    TEST_ASSERT_GREATER_THAN(0, wkId);
    TEST_ASSERT_GREATER_THAN(wkId, uwId);

    Worker::send({ wkId, worker.START });
    TEST_ASSERT_GREATER_OR_EQUAL(1, worker.calls); // might already be > 1

    int n = 0;
    while (!worker.txDone) { asm ("wfi"); ++n; }
    TEST_ASSERT_EQUAL(28, n);
    while (!worker.rxDone) { asm ("wfi"); ++n; }
    TEST_ASSERT_EQUAL(29, n);

    TEST_ASSERT_EQUAL(31, worker.calls);
    TEST_ASSERT_EQUAL(27*28/2, worker.sum);
    Worker::showStats();
}

void allTests () {
    RUN_TEST(testJumper);
    RUN_TEST(testPoll);
    RUN_TEST(testSync);
    RUN_TEST(testWait); // async in blocking mode (sync-like)
    RUN_TEST(testWork); // async in full non-blocking mode
    RUN_TEST(testSync); // make sure reinit works
    RUN_TEST(testPoll); // make sure reinit works
    RUN_TEST(testLoop);
    RUN_TEST(testSync); // make sure reinit works
    RUN_TEST(testPoll); // make sure reinit works
}
