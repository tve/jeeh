// DMA-based SPI tests.

#include "common.h"
Pin pins [8];
#include <jee/dev/flash.h>
#include "defs.h"

constexpr auto MARGIN = 10000; // non-zero loosens microsecond timing checks
constexpr auto SPEED = 10'000; // SPI bus speed, kHz

const uint8_t expect [] = { 0xE6,0x67,0x64,0xA5,0x53,0x5C,0x73,0x23 }; // serno

Ticker ticker;
TICKER_TRIGGER(ticker)

Dev<spi::Gpio<SPI_CONF>> spiGpio;
Dev<spi::Poll<SPI_CONF>> spiPoll;
Dev<spi::Sync<SPI_CONF>> spiSync;

Dev<spi::Async<SPI_CONF>> spiAsync;
SPI_TRIGGER(spiAsync)

void setUp () {}

void tearDown () {
    spiGpio.deinit();
    spiPoll.deinit();
    spiSync.deinit();
    spiAsync.deinit();
}

void testTxGpio () {
    spiGpio.init(SPEED);

    auto start = cycles::micros();
    spiGpio.write("x", 1);
    TEST_ASSERT_INT_WITHIN(MARGIN, 7, cycles::micros()-start);

    start = cycles::micros();
    spiGpio.write("abcde", 5);
    TEST_ASSERT_INT_WITHIN(MARGIN, 29, cycles::micros()-start);

    start = cycles::micros();
    spiGpio.write("1234567890", 10);
    TEST_ASSERT_INT_WITHIN(MARGIN, 57, cycles::micros()-start);

    start = cycles::micros();
    spiGpio.write("123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(MARGIN, 174, cycles::micros()-start);
}

void testRxGpio () {
    spiGpio.init(SPEED);

    uint8_t buf [100];
    auto start = cycles::micros();
    spiGpio.read(buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(MARGIN, 577, cycles::micros()-start);
}

void testFlashGpio () {
    spiGpio.init(SPEED);
    SpiFlash spif (spiGpio);

    // expect a W25Q16 chip of 2 MB, serial# 0xE66764A5535C7323

    TEST_ASSERT_EQUAL_HEX(0xEF4015, spif.info());
    TEST_ASSERT_EQUAL(2048, spif.size());

    uint8_t snBuf [8];
    spif.serNum(snBuf);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, snBuf, sizeof snBuf);

    auto start = cycles::millis();
    spif.erase(0);
    TEST_ASSERT_INT_WITHIN(MARGIN, 33, cycles::millis()-start);

    uint8_t buf [512], buf2 [512];
    memset(buf, 0x55, sizeof buf);

    start = cycles::millis();
    spif.write(0, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(MARGIN, 4, cycles::millis()-start);

    spif.read(0, buf2, sizeof buf2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf2);
}

void testTxPoll () {
    spiPoll.init(SPEED);

    auto start = cycles::micros();
    spiPoll.write("x", 1);
    TEST_ASSERT_INT_WITHIN(MARGIN, 1, cycles::micros()-start);

    start = cycles::micros();
    spiPoll.write("abcde", 5);
    TEST_ASSERT_INT_WITHIN(MARGIN, 2, cycles::micros()-start);

    start = cycles::micros();
    spiPoll.write("1234567890", 10);
    TEST_ASSERT_INT_WITHIN(MARGIN, 3, cycles::micros()-start);

    start = cycles::micros();
    spiPoll.write("123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(MARGIN, 7, cycles::micros()-start);
}

void testRxPoll () {
    spiPoll.init(SPEED);

    uint8_t buf [100];
    auto start = cycles::micros();
    spiPoll.read(buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(MARGIN, 23, cycles::micros()-start);
}

void testFlashPoll () {
    spiPoll.init(SPEED);
    SpiFlash spif (spiPoll);

    // expect a W25Q16 chip of 2 MB, serial# 0xE66764A5535C7323

    TEST_ASSERT_EQUAL_HEX(0xEF4015, spif.info());
    TEST_ASSERT_EQUAL(2048, spif.size());

    uint8_t snBuf [8];
    spif.serNum(snBuf);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, snBuf, sizeof snBuf);

    auto start = cycles::millis();
    spif.erase(0);
    TEST_ASSERT_INT_WITHIN(MARGIN, 31, cycles::millis()-start);

    uint8_t buf [512], buf2 [512];
    memset(buf, 0x55, sizeof buf);

    start = cycles::millis();
    spif.write(0, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(MARGIN, 1, cycles::millis()-start);

    spif.read(0, buf2, sizeof buf2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf2);
}

void testTxSync () {
    spiSync.init(SPEED);

    auto start = cycles::micros();
    spiSync.write("x", 1);
    TEST_ASSERT_INT_WITHIN(MARGIN, 2, cycles::micros()-start);

    start = cycles::micros();
    spiSync.write("abcde", 5);
    TEST_ASSERT_INT_WITHIN(MARGIN, 2, cycles::micros()-start);

    start = cycles::micros();
    spiSync.write("1234567890", 10);
    TEST_ASSERT_INT_WITHIN(MARGIN, 3, cycles::micros()-start);

    start = cycles::micros();
    spiSync.write("123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(MARGIN, 8, cycles::micros()-start);
}

void testRxSync () {
    spiSync.init(SPEED);

    uint8_t buf [100];
    auto start = cycles::micros();
    spiSync.read(buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(MARGIN, 21, cycles::micros()-start);
}

void testFlashSync () {
    spiSync.init(SPEED);
    SpiFlash spif (spiSync);

    // expect a W25Q16 chip of 2 MB, serial# 0xE66764A5535C7323

    TEST_ASSERT_EQUAL_HEX(0xEF4015, spif.info());
    TEST_ASSERT_EQUAL(2048, spif.size());

    uint8_t snBuf [8];
    spif.serNum(snBuf);
logDump(snBuf, sizeof snBuf);
    //TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, snBuf, sizeof snBuf);

    auto start = cycles::millis();
    spif.erase(0);
    TEST_ASSERT_INT_WITHIN(MARGIN, 28, cycles::millis()-start);

    uint8_t buf [512], buf2 [512];
    memset(buf, 0x55, sizeof buf);

    start = cycles::millis();
    spif.write(0, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(MARGIN, 1, cycles::millis()-start);

    spif.read(0, buf2, sizeof buf2);
//logDump(buf2, 16);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf2);
}

void testTxWait () {
    spiAsync.init(SPEED);

    auto start = cycles::micros();
    spiAsync.write("x", 1);
    TEST_ASSERT_INT_WITHIN(MARGIN, 8, cycles::micros()-start);

    start = cycles::micros();
    spiAsync.write("abcde", 5);
    TEST_ASSERT_INT_WITHIN(MARGIN, 3, cycles::micros()-start);

    start = cycles::micros();
    spiAsync.write("1234567890", 10);
    TEST_ASSERT_INT_WITHIN(MARGIN, 3, cycles::micros()-start);

    start = cycles::micros();
    spiAsync.write("123456789012345678901234567890", 30);
    TEST_ASSERT_INT_WITHIN(MARGIN, 12, cycles::micros()-start);
}

void testRxWait () {
    spiAsync.init(SPEED);

    uint8_t buf [100];
    auto start = cycles::micros();
    spiAsync.read(buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(MARGIN, 29, cycles::micros()-start);
}

void testFlashWait () {
    spiAsync.init(SPEED);
    SpiFlash spif (spiAsync);

    // expect a W25Q16 chip of 2 MB, serial# 0xE66764A5535C7323

    TEST_ASSERT_EQUAL_HEX(0xEF4015, spif.info());
    TEST_ASSERT_EQUAL(2048, spif.size());

    uint8_t snBuf [8];
    spif.serNum(snBuf);
logDump(snBuf, sizeof snBuf);
    //TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, snBuf, sizeof snBuf);

    auto start = cycles::millis();
    spif.erase(0);
    TEST_ASSERT_INT_WITHIN(MARGIN, 28, cycles::millis()-start);

    uint8_t buf [512], buf2 [512];
    memset(buf, 0x55, sizeof buf);

    start = cycles::millis();
    spif.write(0, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(MARGIN, 1, cycles::millis()-start);

    spif.read(0, buf2, sizeof buf2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf2);
}

struct SpiTask : Task {
    enum TAG { START, TX, TX1, TX2, TX3, RX, DONE };

    uint8_t calls =0;
    bool done =false;
    uint8_t buf [100];

    using Task::init;

private:
    Event process (Event in, Event out) override {
        ++calls;

        switch (in.eTag) {
            case START:
                break;
            case TX:
                spiAsync.setReply({ tId, TX1 });
                spiAsync.write("x", 1);
                break;
            case TX1:
                spiAsync.setReply({ tId, TX2 });
                spiAsync.write("abcde", 5);
                break;
            case TX2:
                spiAsync.setReply({ tId, TX3 });
                spiAsync.write("1234567890", 10);
                break;
            case TX3:
                spiAsync.setReply({ tId, DONE });
                spiAsync.write("123456789012345678901234567890", 30);
                break;
            case RX:
                memset(buf, 0, sizeof buf);
                spiAsync.setReply({ tId, DONE });
                spiAsync.read(buf, sizeof buf);
                break;
            case DONE:
                done = true;
                break;
            default:
                fail();
        }
        return out;
    }
};

void testTxAsync () {
    SpiTask task;
    auto swId = spiAsync.init(SPEED);
    auto wkId = task.init();

    TEST_ASSERT_GREATER_THAN(0, wkId);
    TEST_ASSERT_GREATER_THAN(wkId, swId);

    auto start = cycles::micros();
    Task::send({ wkId, task.TX });
    TEST_ASSERT_GREATER_OR_EQUAL(1, task.calls); // might already be 2

    int n = 0;
    while (!task.done) { asm ("wfi"); ++n; }
    TEST_ASSERT_INT_WITHIN(MARGIN, 50, cycles::micros()-start);

    TEST_ASSERT_EQUAL(5, task.calls);
}

void testRxAsync () {
    SpiTask task;
    auto swId = spiAsync.init(SPEED);
    auto wkId = task.init();

    TEST_ASSERT_GREATER_THAN(0, wkId);
    TEST_ASSERT_GREATER_THAN(wkId, swId);

    auto start = cycles::micros();
    Task::send({ wkId, task.RX });
    TEST_ASSERT_EQUAL(1, task.calls);

    int n = 0;
    while (!task.done) { asm ("wfi"); ++n; }
    TEST_ASSERT_INT_WITHIN(MARGIN, 38, cycles::micros()-start);

    TEST_ASSERT_EQUAL(2, task.calls);
}

void testFlashAsync () {
    SpiTask task;
    auto swId = spiAsync.init(SPEED);
    auto wkId = task.init();

    TEST_ASSERT_GREATER_THAN(0, wkId);
    TEST_ASSERT_GREATER_THAN(wkId, swId);

    // TODO flash driver will need to be extended to work in async mode
    //  i.e. wrap as task and use periodic ticks to check erase completion

    SpiFlash spif (spiAsync);

    // expect a W25Q16 chip of 2 MB, serial# 0xE66764A5535C7323

    TEST_ASSERT_EQUAL_HEX(0xEF4015, spif.info());
    TEST_ASSERT_EQUAL(2048, spif.size());

    uint8_t snBuf [8];
    spif.serNum(snBuf);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, snBuf, sizeof snBuf);

    auto start = cycles::millis();
    spif.erase(0);
    TEST_ASSERT_INT_WITHIN(MARGIN, 28, cycles::millis()-start);

    uint8_t buf [512], buf2 [512];
    memset(buf, 0x55, sizeof buf);

    start = cycles::millis();
    spif.write(0, buf, sizeof buf);
    TEST_ASSERT_INT_WITHIN(MARGIN, 1, cycles::millis()-start);

    spif.read(0, buf2, sizeof buf2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(buf, buf2, sizeof buf2);
}

void allTests () {
    Pin::config("A15:U,B7,B5,B4,A11,B3,A1:P,A0", pins, sizeof pins);
    for (auto e : pins) e = 1;
    pins[5] = 0; // ~SCLK

    RUN_TEST(testTxGpio);
    RUN_TEST(testRxGpio);
    RUN_TEST(testFlashGpio);
    RUN_TEST(testTxPoll);
    RUN_TEST(testRxPoll);
    RUN_TEST(testFlashPoll);
    RUN_TEST(testTxSync);
    RUN_TEST(testRxSync);
    RUN_TEST(testFlashSync);
    RUN_TEST(testTxWait);
    RUN_TEST(testRxWait);
pins[6] = 0;
    RUN_TEST(testFlashWait); // async in blocking mode (sync-like)
pins[7] = 0;
    RUN_TEST(testTxAsync);
    RUN_TEST(testRxAsync);
    //RUN_TEST(testFlashAsync);
}
