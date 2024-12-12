// DMA-based UART tests.

#include "common.h"
#include "defs.h"

constexpr auto MARGIN = 10000; // non-zero loosens microsecond timing checks

Ticker ticker;
TICKER_TRIGGER(ticker)

Dev<uart::Poll<UART_CONF>> uartPoll;
Dev<uart::Sync<UART_CONF>> uartSync;

Dev<uart::Async<UART_CONF>> uartAsync;
UART_TRIGGER(uartAsync)

void setUp () {}

void tearDown () {
    uartPoll.deinit();
    uartSync.deinit();
    uartAsync.deinit();
}

void testJumper () {
    Pin outPin ("A9","P"), inPin ("A10","F");

    // check that the two pins are connected via a jumper
    TEST_ASSERT_EQUAL(0, inPin);
    outPin = 1;
    TEST_ASSERT_EQUAL(1, inPin);
}

void testPoll () {
    uartPoll.init(1'000'000);

    auto start = cycles::micros();
    uartPoll.write("x", 1);
    TEST_ASSERT_INT_WITHIN(MARGIN, 20, cycles::micros()-start);

    start = cycles::micros();
    uartPoll.write("abcde", 5);
    TEST_ASSERT_INT_WITHIN(MARGIN, 52, cycles::micros()-start);

    start = cycles::micros();
    uartPoll.write("1234567890", 10);
    TEST_ASSERT_INT_WITHIN(MARGIN, 102, cycles::micros()-start);

    start = cycles::micros();
    uartPoll.write("123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(MARGIN, 307, cycles::micros()-start);
}

void testSync () {
    uartSync.init(1'000'000);

    auto start = cycles::micros();
    uartSync.write("x", 1);
    TEST_ASSERT_INT_WITHIN(MARGIN, 2, cycles::micros()-start);

    start = cycles::micros();
    uartSync.write("abcde", 5);
    TEST_ASSERT_INT_WITHIN(MARGIN, 47, cycles::micros()-start);

    start = cycles::micros();
    uartSync.write("1234567890", 10);
    TEST_ASSERT_INT_WITHIN(MARGIN, 100, cycles::micros()-start);

    start = cycles::micros();
    uartSync.write("123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(MARGIN, 300, cycles::micros()-start);
}

void testWait () {
    uartAsync.init(1'000'000);

    auto start = cycles::micros();
    uartAsync.write("x", 1);
    TEST_ASSERT_INT_WITHIN(MARGIN, 3, cycles::micros()-start);

    start = cycles::micros();
    uartAsync.write("abcde", 5);
    TEST_ASSERT_INT_WITHIN(MARGIN, 51, cycles::micros()-start);

    start = cycles::micros();
    uartAsync.write("1234567890", 10);
    TEST_ASSERT_INT_WITHIN(MARGIN, 99, cycles::micros()-start);

    start = cycles::micros();
    uartAsync.write("123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(MARGIN, 300, cycles::micros()-start);
}

struct UartTask : Task {
    enum TAG { START, ONE, TWO, THREE, FOUR };

    uint8_t calls =0;
    bool done =false;

    using Task::init;

private:
    Event process (Event in, Event out) override {
        ++calls;

        switch (in.eTag) {
            case START:
                uartAsync.setReply({ tId, ONE });
                uartAsync.write("x", 1);
                break;
            case ONE:
                uartAsync.setReply({ tId, TWO });
                uartAsync.write("abcde", 5);
                break;
            case TWO:
                uartAsync.setReply({ tId, THREE });
                uartAsync.write("1234567890", 10);
                break;
            case THREE:
                uartAsync.setReply({ tId, FOUR });
                uartAsync.write("123456789012345678901234567890", 30);
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

void testAsync () {
    UartTask task;
    auto uwId = uartAsync.init(1'000'000);
    auto wkId = task.init();

    TEST_ASSERT_GREATER_THAN(0, wkId);
    TEST_ASSERT_GREATER_THAN(wkId, uwId);

    auto start = cycles::micros();
    Task::send({ wkId, task.START });
    TEST_ASSERT_GREATER_OR_EQUAL(1, task.calls); // might already be 2

    int n = 0;
    while (!task.done) { asm ("wfi"); ++n; }
    TEST_ASSERT_INT_WITHIN(MARGIN, 456, cycles::micros()-start); // 1+5+10+30 ch

    TEST_ASSERT_EQUAL(5, task.calls);
}

struct LoopTask : Task {
    enum TAG { START, MORE, SENT, RECV };

    uint8_t calls =0, count =0;
    uint16_t sum =0;
    bool txDone =false, rxDone =false;

    using Task::init;

private:
    Event process (Event in, Event out) override {
        ++calls;

        switch (in.eTag) {
            case START:
                uartAsync.setReply({ tId, RECV });
                uartAsync.read(nullptr, 0);
                [[fallthrough]];
            case MORE:
                ++count;
                // send 1 + 2 + 3 + ... + 25 + 26 + 27 bytes
                uartAsync.setReply({ tId, count < 27 ? MORE : SENT });
                uartAsync.write("~ABCDEFGHIJKLMNOPQRSTUVWXYZ", count);
                break;
            case SENT:
                txDone = true;
                break;
            case RECV:
                // count the number of bytes received
                sum += in.eVal;
                if (sum >= 27*28/2) {
                    uartAsync.read(nullptr, in.eVal); // consume without new request
                    rxDone = true;
                } else { // keep reading
                    uartAsync.setReply({ tId, RECV });
                    uartAsync.read(nullptr, in.eVal);
                }
                break;
            default:
                fail();
        }
        return out;
    }
};

void testLoop () {
    LoopTask task;
    auto uwId = uartAsync.init(1'000'000);
    auto wkId = task.init();

    TEST_ASSERT_GREATER_THAN(0, wkId);
    TEST_ASSERT_GREATER_THAN(wkId, uwId);

    Task::send({ wkId, task.START });
    TEST_ASSERT_GREATER_OR_EQUAL(1, task.calls); // might already be > 1

    int n = 0;
    while (!task.txDone) { asm ("wfi"); ++n; }
    TEST_ASSERT_EQUAL(28, n);
    while (!task.rxDone) { asm ("wfi"); ++n; }
    TEST_ASSERT_EQUAL(29, n);

    TEST_ASSERT_EQUAL(31, task.calls);
    TEST_ASSERT_EQUAL(27*28/2, task.sum);
    Task::showStats();
}

void allTests () {
    RUN_TEST(testJumper);
    RUN_TEST(testPoll);
    RUN_TEST(testSync);
    RUN_TEST(testWait); // async in blocking mode (sync-like)
    RUN_TEST(testAsync); // async in full non-blocking mode
    RUN_TEST(testSync); // make sure reinit works
    RUN_TEST(testPoll); // make sure reinit works
    RUN_TEST(testLoop);
    RUN_TEST(testSync); // make sure reinit works
    RUN_TEST(testPoll); // make sure reinit works
}
